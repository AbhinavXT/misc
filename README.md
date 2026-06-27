{
  "$schema": "https://opencode.ai/config.json",
  "provider": {
    "ollama": {
      "npm": "@ai-sdk/openai-compatible",
      "name": "Ollama (local)",
      "options": { "baseURL": "http://localhost:11434/v1" },
      "models": {
        "gemma4:26b-a4b-64k": {
          "name": "Gemma 4 26B-A4B (64k)",
          "tools": true
        }
      }
    }
  }
}