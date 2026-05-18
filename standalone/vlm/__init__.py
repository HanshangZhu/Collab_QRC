"""VLM-driven exploration for the standalone MuJoCo stack.

Modules:
    backend  — pluggable LLM HTTP client (xAI / OpenAI / Anthropic / mock)
    renderer — OccupancyMapper grid + robot pose → PNG bytes for VLM input
    prompts  — system + user prompt builders, JSON response schema
    explorer — VLMExplorer node: subscribes map/odom, calls VLM at interval,
               publishes goals onto the same EventBus topics CFPA2 used
"""
