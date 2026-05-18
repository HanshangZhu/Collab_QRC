"""Pluggable LLM HTTP backend for standalone VLM exploration.

Stdlib-only HTTP client (urllib). No ROS, no third-party SDKs.

Providers:
    xai       — Grok via x.ai (OpenAI-compatible /chat/completions)
    openai    — GPT-4o / GPT-4V via api.openai.com
    anthropic — Claude via api.anthropic.com
    google    — Gemini / Gemma via Google AI Studio OpenAI-compatible endpoint
    mock      — no network; deterministic stub for tests

API key resolution order:
    1. explicit `api_key` argument
    2. env var: XAI_API_KEY / OPENAI_API_KEY / ANTHROPIC_API_KEY / GEMINI_API_KEY
    3. .env files at repo root: .env, .env.xai, .env.openai, .env.anthropic, .env.google
       Format: KEY=value, one per line; blank + #-comment lines ignored.
"""
from __future__ import annotations

import json
import os
import re
import urllib.error
import urllib.request
from dataclasses import dataclass
from pathlib import Path
from typing import Optional


_REPO_ROOT = Path(__file__).resolve().parents[2]
_DOTENV_FILES = (".env", ".env.xai", ".env.openai", ".env.anthropic",
                 ".env.google", ".env.gemini", ".env.groq", ".env.local")
_dotenv_loaded = False


class VLMBackendError(RuntimeError):
    pass


def _load_dotenv_once() -> None:
    """Lazy-load .env files at repo root into os.environ if not already set."""
    global _dotenv_loaded
    if _dotenv_loaded:
        return
    _dotenv_loaded = True
    for name in _DOTENV_FILES:
        path = _REPO_ROOT / name
        if not path.is_file():
            continue
        try:
            with open(path) as f:
                for line in f:
                    line = line.strip()
                    if not line or line.startswith("#") or "=" not in line:
                        continue
                    k, _, v = line.partition("=")
                    k = k.strip()
                    v = v.strip().strip("'\"")
                    if k and k not in os.environ:
                        os.environ[k] = v
        except Exception:
            pass


def _clean(s: str) -> str:
    return "".join(s.split()) if s else ""


def resolve_provider(provider: str) -> str:
    """Resolve 'auto' → first provider with a key set; else passthrough."""
    _load_dotenv_once()
    p = (provider or "").strip().lower()
    if p == "gemini":
        p = "google"
    if p and p != "auto":
        return p
    # Auto-resolution priority: xAI → Groq → Google → OpenAI → Anthropic → mock
    # xAI first because it's already used in this project (door task).
    # Groq before Google because Groq has no daily quota exhaustion on free tier.
    if os.environ.get("XAI_API_KEY"):
        return "xai"
    if os.environ.get("GROQ_API_KEY"):
        return "groq"
    if os.environ.get("GEMINI_API_KEY") or os.environ.get("GOOGLE_API_KEY"):
        return "google"
    if os.environ.get("OPENAI_API_KEY"):
        return "openai"
    if os.environ.get("ANTHROPIC_API_KEY"):
        return "anthropic"
    return "mock"


def resolve_api_key(provider: str) -> str:
    _load_dotenv_once()
    p = resolve_provider(provider)
    if p == "xai":
        return _clean(os.environ.get("XAI_API_KEY", ""))
    if p == "openai":
        return _clean(os.environ.get("OPENAI_API_KEY", ""))
    if p == "anthropic":
        return _clean(os.environ.get("ANTHROPIC_API_KEY", ""))
    if p == "google":
        return _clean(os.environ.get("GEMINI_API_KEY") or
                      os.environ.get("GOOGLE_API_KEY", ""))
    if p == "groq":
        return _clean(os.environ.get("GROQ_API_KEY", ""))
    return ""


def default_model(provider: str) -> str:
    p = resolve_provider(provider)
    return {
        "xai":      "grok-4-1-fast-non-reasoning",
        "openai":   "gpt-4o-mini",
        "anthropic": "claude-haiku-4-5-20251001",
        "google":   "gemini-2.0-flash",
        "groq":     "meta-llama/llama-4-scout-17b-16e-instruct",
        "mock":     "mock-vlm",
    }.get(p, "")


# ── JSON extraction ──────────────────────────────────────────────────────────

def extract_json_object(raw: str) -> Optional[dict]:
    """Pull the first JSON object out of a model response.

    Handles:
      - markdown ```...``` fences
      - <thought>...</thought> blocks (Gemma 4 internal reasoning)
      - <reasoning>...</reasoning> blocks
      - leading/trailing prose
    """
    text = (raw or "").strip()
    # Strip XML-ish reasoning blocks before JSON detection.
    text = re.sub(r"<thought>.*?</thought>", "", text, flags=re.DOTALL | re.IGNORECASE)
    text = re.sub(r"<reasoning>.*?</reasoning>", "", text, flags=re.DOTALL | re.IGNORECASE)
    # Strip a leading unclosed <thought> block (e.g. if max_tokens cut off mid-thought).
    text = re.sub(r"^<thought>.*", "", text, flags=re.DOTALL | re.IGNORECASE).strip()
    if text.startswith("```"):
        lines = text.splitlines()
        if len(lines) >= 2:
            text = "\n".join(lines[1:])
        if text.endswith("```"):
            text = text[:-3]
    try:
        return json.loads(text.strip())
    except json.JSONDecodeError:
        # Prefer the LAST balanced {...} block (final answer after reasoning).
        candidates = list(re.finditer(r"\{[^{}]*(?:\{[^{}]*\}[^{}]*)*\}",
                                       text, re.DOTALL))
        for match in reversed(candidates):
            try:
                return json.loads(match.group(0))
            except json.JSONDecodeError:
                continue
        # Last-ditch greedy: first { to last }
        first = text.find("{")
        last = text.rfind("}")
        if first >= 0 and last > first:
            try:
                return json.loads(text[first:last + 1])
            except json.JSONDecodeError:
                return None
        return None


# ── HTTP ─────────────────────────────────────────────────────────────────────

def _post_json(url: str, payload: dict, headers: dict,
               timeout_sec: float) -> dict:
    req = urllib.request.Request(
        url=url,
        data=json.dumps(payload).encode("utf-8"),
        headers=headers,
        method="POST",
    )
    try:
        with urllib.request.urlopen(req, timeout=timeout_sec) as resp:
            return json.loads(resp.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        detail = exc.read().decode("utf-8", errors="replace")
        raise VLMBackendError(f"HTTP {exc.code}: {detail[:600]}") from exc
    except urllib.error.URLError as exc:
        raise VLMBackendError(f"Request failed: {exc.reason}") from exc


# ── Mock ─────────────────────────────────────────────────────────────────────

@dataclass
class MockResult:
    """Deterministic frontier-picker fallback when no API key is set.

    Picks the centroid of the largest unknown-adjacent free patch in the
    grid. Output matches the JSON contract used for the real VLM call.
    """
    goal_x: float
    goal_y: float
    reason: str = "mock: nearest large frontier"


def _mock_pick_goal(scene: dict) -> str:
    """Picks a goal from scene's frontier_candidates list.

    Scene includes a list of candidate frontier (x, y, info_gain). The mock
    just picks the highest-info candidate to mimic a sensible explorer.
    """
    cands = scene.get("frontier_candidates", [])
    if not cands:
        return json.dumps({"goal": None, "reason": "mock: no frontiers"})
    best = max(cands, key=lambda c: c.get("info_gain", 0.0))
    return json.dumps({
        "goal": {"x": float(best["x"]), "y": float(best["y"])},
        "reason": f"mock: pick highest info gain={best.get('info_gain', 0):.2f}",
    })


# ── Main entry ───────────────────────────────────────────────────────────────

def query_vlm(
    *,
    provider: str,
    model: str,
    system_prompt: str,
    user_prompt: str,
    image_b64: Optional[str] = None,
    temperature: float = 0.1,
    max_tokens: int = 768,
    timeout_sec: float = 30.0,
    scene_for_mock: Optional[dict] = None,
) -> str:
    """Send a single VLM round-trip; return the raw response text.

    Caller is responsible for parsing JSON out of the response.
    For provider=='mock', returns a deterministic JSON string instead of
    a network call; `scene_for_mock` is used to pick a goal.
    """
    p = resolve_provider(provider)

    if p == "mock":
        return _mock_pick_goal(scene_for_mock or {})

    api_key = resolve_api_key(p)
    if not api_key:
        raise VLMBackendError(f"No API key set for provider '{p}'")

    model = model or default_model(p)

    if p == "anthropic":
        content = []
        if image_b64:
            content.append({
                "type": "image",
                "source": {
                    "type": "base64",
                    "media_type": "image/png",
                    "data": image_b64,
                },
            })
        content.append({"type": "text", "text": user_prompt})
        payload = {
            "model": model,
            "max_tokens": max_tokens,
            "system": system_prompt,
            "temperature": temperature,
            "messages": [{"role": "user", "content": content}],
        }
        body = _post_json(
            "https://api.anthropic.com/v1/messages",
            payload,
            {
                "Content-Type": "application/json",
                "x-api-key": api_key,
                "anthropic-version": "2023-06-01",
            },
            timeout_sec,
        )
        chunks = body.get("content", [])
        if not chunks:
            raise VLMBackendError("Anthropic response empty")
        return str(chunks[0].get("text", ""))

    if p in {"openai", "xai", "google"}:
        if p == "xai":
            url = os.environ.get("XAI_API_BASE_URL", "https://api.x.ai/v1").rstrip("/") + "/chat/completions"
        elif p == "google":
            # Google AI Studio's OpenAI-compatible endpoint. Same /chat/completions
            # shape; accepts image_url with data: URIs just like OpenAI.
            url = (os.environ.get(
                "GOOGLE_API_BASE_URL",
                "https://generativelanguage.googleapis.com/v1beta/openai",
            ).rstrip("/") + "/chat/completions")
        else:
            url = "https://api.openai.com/v1/chat/completions"
        user_content = []
        if image_b64:
            user_content.append({
                "type": "image_url",
                "image_url": {
                    "url": f"data:image/png;base64,{image_b64}",
                    "detail": "low",
                },
            })
        user_content.append({"type": "text", "text": user_prompt})
        payload = {
            "model": model,
            "temperature": temperature,
            "max_tokens": max_tokens,
            "messages": [
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": user_content},
            ],
        }
        body = _post_json(
            url,
            payload,
            {"Content-Type": "application/json",
             "Authorization": f"Bearer {api_key}"},
            timeout_sec,
        )
        choices = body.get("choices", [])
        if not choices:
            raise VLMBackendError(f"{p} response empty")
        return str(choices[0].get("message", {}).get("content", ""))

    raise VLMBackendError(f"Unsupported provider '{p}'")


# ── CLI helper: list available models (Google only — most providers don't expose) ──

def list_google_models(api_key: str) -> list[dict]:
    """Hit Google's models endpoint, return list of {id, display_name, methods}."""
    url = (os.environ.get(
        "GOOGLE_API_BASE_URL",
        "https://generativelanguage.googleapis.com/v1beta/openai",
    ).rstrip("/") + "/models")
    req = urllib.request.Request(
        url=url,
        headers={"Authorization": f"Bearer {api_key}"},
        method="GET",
    )
    try:
        with urllib.request.urlopen(req, timeout=15.0) as resp:
            body = json.loads(resp.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        detail = exc.read().decode("utf-8", errors="replace")
        raise VLMBackendError(f"HTTP {exc.code}: {detail[:600]}") from exc
    return body.get("data", []) or body.get("models", [])


def _cli() -> None:
    import argparse
    ap = argparse.ArgumentParser(description="VLM backend helper")
    ap.add_argument("--list-models", action="store_true")
    ap.add_argument("--provider", default="google")
    args = ap.parse_args()
    if not args.list_models:
        ap.print_help()
        return
    p = resolve_provider(args.provider)
    if p != "google":
        print(f"--list-models currently only supported for provider=google "
              f"(got {p})")
        return
    key = resolve_api_key("google")
    if not key:
        print("No GEMINI_API_KEY / GOOGLE_API_KEY in env or .env files")
        return
    models = list_google_models(key)
    print(f"# {len(models)} models available via Google AI Studio OpenAI-compat:")
    for m in models:
        mid = m.get("id") or m.get("name", "")
        owned = m.get("owned_by", "")
        print(f"  {mid}\t{owned}")


if __name__ == "__main__":
    _cli()
