# NeoPixel Studio

<p align="center">
  <img src="logo.png" width="450" alt="NeoPixel Studio Logo">
</p>

<p align="center">
  <b>ESP8266 (NodeMCU ESP-12E) firmware + embedded web UI for WS2812B / NeoPixel strips.</b><br>
  The board serves the whole interface from flash and streams live control over a WebSocket.
</p>

---

## 📸 Interface Modes

NeoPixel Studio features two web-based UI modes tailored for quick control or complex customization:

| Simple Mode (with AI Chat) | Advanced Mode (Layer Stack & Controls) |
| :---: | :---: |
| <img src="screenshot_simple.png" width="400" alt="Simple Mode Interface"> | <img src="screenshot_advanced.png" width="400" alt="Advanced Mode Interface"> |

### 🟢 Simple Mode
Designed for effortless control and quick adjustments:
* **Ask the AI Panel:** Natural language interface to generate multi-layer lighting shows directly via LLM prompts.
* **Instant Presets:** One-click quick actions and colors for immediate playback.
* **Live Interactive Preview:** Real-time visual feedback before and during hardware updates.
* **One-Click Push & Undo:** Instantly send effect plans to the ESP8266 or step back with the built-in undo history.

### ⚙️ Advanced Mode
Full creative control over the LED strip hardware and animation rendering engine:
* **Multi-Layer Stack & Blending:** Stack multiple animation effects concurrently with custom blend modes (Over, Add, Multiply, Screen, Mask, etc.).
* **Fine-Tuned Parameter Controls:** Deep control over color palettes, animation speeds, direction, phase shifts, pixel ranges, and global brightness.
* **16 Custom Effect Generators:** Includes Rainbow, Color Wipe, Breathing, Sparkle, Fire, Running Light, Theater Chase, Twinkle, Strobe, Solid Color, Custom Pixel Mapping, and more.
* **Live Pixel-by-Pixel Mapping:** Manually target individual LEDs or sub-ranges across the strip for precision patterns.

---

## Repository layout

| File | What it is |
| :--- | :--- |
| `NeoPixelStudio.ino` | Firmware: web server, WebSocket link, effect engine, playback |
| `web/index.html` | **Source of truth for the web UI** (single file, no build step) |
| `ui_embedded.h` | Auto-generated: the UI as a PROGMEM C string. Do not edit |
| `tools/build_embedded_ui.py` | Encodes `web/index.html` → `ui_embedded.h` |
| `tools/decode_ui.py` | Extracts `ui_embedded.h` → `web/index.html` (for recovery) |

After editing `web/index.html`, regenerate the header and reflash:

```bash
python tools/build_embedded_ui.py
```

---

## AI chat (Simple mode)

The Simple tab has an **Ask the AI** panel: describe the show in plain English and the assistant converts it into the app's effect stack, plays it on the live preview immediately, and pushes it to the board when connected. Each applied plan shows what was built with an **Undo** button.

### Examples:

- *"light 1st 8 leds in red, next 8 in blue"*
- *"blink 1-4 leds in red with 1 seconds delay, blink rest in yellow with 3 seconds delay"*
- *"light 1st 6 leds with red, next 4 in yellow, last 6 in red"*

### How it works

- The browser talks **directly** to the provider's chat-completions API (BYOK). The ESP8266 never sees the API key and cannot proxy TLS anyway.
- The system prompt describes the 16 effects, the layer/blend model and the bpm→seconds maths (blink period = 30/bpm s), so "1 second delay" becomes bpm 30.
- The reply is a small JSON plan (`layers`, optional `brightness` / `globalSpeed`), converted with the same `mkEffect` / `mkSolid` / `mkPixels` factories the rest of the UI uses. Multiple "pixels" groups merge into one pixel layer because the device protocol carries pixel data for a single layer.
- Keys and per-provider settings live in `localStorage` (`nps-ai`) only.

### Supported providers

OpenAI, Anthropic (Claude), Google Gemini, **DeepInfra** (default), OpenRouter, Groq, Mistral, DeepSeek, xAI (Grok), Together AI, Fireworks, Cerebras, Perplexity, Cohere, Hugging Face Router, NVIDIA NIM, Alibaba Qwen, Moonshot (Kimi), Zhipu GLM, 01.AI (Yi), Azure OpenAI, Ollama and LM Studio (local, no key), plus any custom OpenAI-compatible endpoint. Base URLs are editable and "Load" fetches the live model list where the provider offers one.

Anthropic calls send `anthropic-dangerous-direct-browser-access` so they work from a browser; providers that block direct browser calls fail with a clear CORS hint in the chat log.