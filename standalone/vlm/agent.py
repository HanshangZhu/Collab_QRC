"""LangGraph ReAct exploration agent.

Uses langgraph.prebuilt.create_react_agent to build a tool-calling loop
around the four exploration tools (list_frontiers, validate_path,
get_coverage, log_artifact). The agent receives:

    - Image 1: top-down occupancy map PNG (gray=unknown, white=free, black=wall)
    - Image 2: robot front-camera RGB frame (optional, may be None)
    - Scene JSON: map extents, robot pose, frontier candidates, past history
    - System prompt: mission + output contract

It can call tools zero or more times before emitting a final JSON response
matching the {goal, reason, artifact_seen, artifact_pos, done} contract.

Provider routing:
    google  → ChatOpenAI → Google AI Studio OpenAI-compat endpoint
    xai     → ChatOpenAI → api.x.ai/v1
    openai  → ChatOpenAI → api.openai.com/v1
    mock    → bypass LangGraph, return deterministic frontier pick

No langchain-google-genai dependency needed — ChatOpenAI handles all three
providers via their OpenAI-compatible /chat/completions interfaces.
"""
from __future__ import annotations

import json
import os
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

from langchain_core.messages import AIMessage, HumanMessage, SystemMessage
from langchain_openai import ChatOpenAI
from langgraph.prebuilt import create_react_agent


_REPO_ROOT = Path(__file__).resolve().parents[2]

_DEFAULT_MODELS: Dict[str, str] = {
    # gemini-2.0-flash: reliable tool-calling via OpenAI-compat endpoint.
    # gemini-2.5-flash: faster/smarter but thinking mode can conflict with
    # tool calling on the compat endpoint — use --model gemini-2.5-flash to
    # override if needed.
    "google": "gemini-2.0-flash",
    "xai":    "grok-4-1-fast-non-reasoning",
    "openai": "gpt-4o-mini",
    # Groq free tier: vision + tool calling. Llama 4 Scout = 17B MoE, fast.
    # Sign up at console.groq.com → free API key → set GROQ_API_KEY in .env
    # Free limits: 30 RPM, 1000 RPD, 6000 TPM. Enough for 12s-cycle exploration.
    "groq":   "meta-llama/llama-4-scout-17b-16e-instruct",
}

_BASE_URLS: Dict[str, str] = {
    "google": "https://generativelanguage.googleapis.com/v1beta/openai/",
    "xai":    "https://api.x.ai/v1/",
    "openai": "https://api.openai.com/v1/",
    "groq":   "https://api.groq.com/openai/v1/",
}

_API_KEY_VARS: Dict[str, Tuple[str, ...]] = {
    "google": ("GEMINI_API_KEY", "GOOGLE_API_KEY"),
    "xai":    ("XAI_API_KEY",),
    "openai": ("OPENAI_API_KEY",),
    "groq":   ("GROQ_API_KEY",),
}


def _resolve_api_key(provider: str) -> str:
    from .backend import _load_dotenv_once
    _load_dotenv_once()
    for var in _API_KEY_VARS.get(provider, ()):
        v = (os.environ.get(var) or "").strip()
        if v:
            return v
    return ""


def build_llm(provider: str, model: str,
              temperature: float = 0.1,
              max_tokens: int = 2048) -> ChatOpenAI:
    """Return a ChatOpenAI instance wired to the correct endpoint."""
    if provider not in _BASE_URLS:
        raise ValueError(
            f"Provider '{provider}' not supported for LangGraph agent. "
            f"Supported: {list(_BASE_URLS)}. Use --provider mock for offline."
        )
    api_key = _resolve_api_key(provider)
    if not api_key:
        raise ValueError(
            f"No API key found for provider '{provider}'. "
            f"Set {_API_KEY_VARS[provider][0]} in .env or environment."
        )
    return ChatOpenAI(
        model=model or _DEFAULT_MODELS.get(provider, "gpt-4o-mini"),
        api_key=api_key,
        base_url=_BASE_URLS[provider],
        temperature=temperature,
        max_tokens=max_tokens,
    )


def _image_block(png_b64: str) -> dict:
    return {
        "type": "image_url",
        "image_url": {"url": f"data:image/png;base64,{png_b64}", "detail": "low"},
    }


def run_agent(
    *,
    provider: str,
    model: str,
    tools: list,
    system_prompt: str,
    map_png_b64: str,
    camera_png_b64: Optional[str],
    user_text: str,
    history: List[dict],
    temperature: float = 0.1,
    max_tokens: int = 2048,
    max_iterations: int = 8,
) -> Dict[str, Any]:
    """Run one LangGraph ReAct cycle.

    Returns a normalized dict:
        {goal, reason, artifact_seen, artifact_pos, done}
    matching the same contract as the old parse_response() output.
    """
    llm = build_llm(provider, model, temperature=temperature, max_tokens=max_tokens)
    agent = create_react_agent(llm, tools)

    # Compose the user HumanMessage:
    #   [map_image, camera_image?, history_lines?, scene_json_text]
    content: list = []

    content.append({"type": "text", "text": "Image 1 — top-down occupancy map:"})
    content.append(_image_block(map_png_b64))

    if camera_png_b64:
        content.append({"type": "text", "text": "Image 2 — robot front camera (RGB):"})
        content.append(_image_block(camera_png_b64))

    # Compact history: last 6 non-null goals so VLM knows where it already went
    goal_history = [h for h in history[-6:] if h.get("goal")]
    if goal_history:
        lines = "\n".join(
            f"  cycle {h['cycle']}: goal=({h['goal'][0]:.2f}, {h['goal'][1]:.2f}) "
            f"reason={h['reason'][:60]!r}"
            for h in goal_history
        )
        content.append({"type": "text",
                        "text": f"Exploration history (past decisions):\n{lines}"})

    content.append({"type": "text", "text": user_text})

    messages = [
        SystemMessage(content=system_prompt),
        HumanMessage(content=content),
    ]

    try:
        result = agent.invoke(
            {"messages": messages},
            config={"recursion_limit": max_iterations * 2 + 1},
        )
    except Exception as exc:
        return {
            "goal": None,
            "reason": f"agent_error: {exc}",
            "artifact_seen": False,
            "artifact_pos": None,
            "done": False,
        }

    # Extract the last non-empty AI text message as the final answer
    final_text = ""
    for msg in reversed(result.get("messages", [])):
        if isinstance(msg, AIMessage):
            text = msg.content if isinstance(msg.content, str) else ""
            # Skip tool-call-only messages (content is empty or a list of tool calls)
            if isinstance(msg.content, list):
                for block in msg.content:
                    if isinstance(block, dict) and block.get("type") == "text":
                        text = block.get("text", "")
                        break
            if text.strip():
                final_text = text
                break

    from .prompts import parse_response

    if not final_text:
        return {
            "goal": None,
            "reason": "empty_agent_response",
            "artifact_seen": False,
            "artifact_pos": None,
            "done": False,
        }

    parsed = parse_response(final_text)

    # If the model returned garbage (e.g. Llama 4 Scout outputs "[][]" when
    # it skips tool calls on early sparse cycles), do ONE direct follow-up
    # asking explicitly for the JSON — no full ReAct loop, just a plain call.
    if "parse_error" in parsed.get("reason", ""):
        import sys
        print(f"[VLM agent] parse_error — raw final_text was: {final_text[:120]!r}",
              file=sys.stderr)
        try:
            followup_msgs = list(result.get("messages", [])) + [
                HumanMessage(
                    content=(
                        "Your previous response could not be parsed as JSON. "
                        "Output ONLY a valid JSON object now — no arrays, no markdown, "
                        "no prose. Start directly with { :\n"
                        '{"goal": {"x": <float>, "y": <float>}, '
                        '"reason": "<short explanation>", '
                        '"artifact_seen": false, "artifact_pos": null, "done": false}'
                    )
                )
            ]
            follow_resp = llm.invoke(followup_msgs)
            retry_text = (
                follow_resp.content
                if isinstance(follow_resp.content, str)
                else ""
            )
            if isinstance(follow_resp.content, list):
                for block in follow_resp.content:
                    if isinstance(block, dict) and block.get("type") == "text":
                        retry_text = block.get("text", "")
                        break
            if retry_text.strip():
                retry_parsed = parse_response(retry_text)
                if "parse_error" not in retry_parsed.get("reason", ""):
                    return retry_parsed
                print(f"[VLM agent] retry also failed — raw: {retry_text[:120]!r}",
                      file=sys.stderr)
        except Exception as exc:
            print(f"[VLM agent] retry exception: {exc}", file=sys.stderr)

    return parsed
