// installer.catpilot.dev — serves the catpilot installer binary.
//
// Paths:
//   /            -> latest stable release (branch name read from R2 object "stable")
//   /dev         -> dev branch
//   /vX.Y.Z      -> that release branch (e.g. /v0.11.1)
//
// Bundled static assets (assets/ directory, served via the ASSETS binding):
//   installer-template         padded aarch64 ELF for current AGNOS (3X, comma four)
//   installer-template-legacy  padded ELF built for old AGNOS (comma three on 12.8)
//   stable                     plain-text branch name the root path serves (e.g. "v0.11.1")
//
// One build covers all device types (tici/tizi/mici branch at runtime); only the
// AGNOS generation matters, so the template is chosen from the setup app's own
// User-Agent ("AGNOSSetup-<os_version>"). Verified against installer.comma.ai:
// same BuildID for tici/tizi/mici at a given AGNOS, different build for 12.8.
//
// The template embeds two '?'-terminated space-padded fields (see openpilot
// selfdrive/ui/installer/installer.cc — "Leave some extra space for the fork
// installer"). We rewrite them per request, exactly like installer.comma.ai does.

const GIT_URL = "https://github.com/catpilot-dev/catpilot.git";
const URL_MARKER = "https://github.com/commaai/openpilot.git?";
const URL_FIELD_LEN = 105;
const BRANCH_MARKER = "release3?";
const BRANCH_FIELD_LEN = 73;

function patchField(buf, latin1, marker, fieldLen, value) {
  // latin1 is a byte-preserving string view of buf; native indexOf is fast.
  const i = latin1.indexOf(marker);
  if (i < 0) throw new Error(`marker not found: ${marker}`);
  const field = new TextEncoder().encode(value + "?");
  if (field.length > fieldLen) throw new Error(`value too long: ${value}`);
  buf.set(field, i);
  buf.fill(0x20, i + field.length, i + fieldLen); // space padding
}

async function branchExists(branch) {
  // Best-effort: if GitHub is unreachable or rate-limited, serve anyway.
  try {
    const r = await fetch(
      `https://api.github.com/repos/catpilot-dev/catpilot/branches/${branch}`,
      { headers: { "User-Agent": "catpilot-installer-worker" },
        cf: { cacheTtl: 300, cacheEverything: true } },
    );
    return r.status === 404 ? false : true;
  } catch {
    return true;
  }
}

async function getAsset(env, name) {
  const r = await env.ASSETS.fetch(`https://assets.internal/${name}`);
  return r.ok ? r : null;
}

export default {
  async fetch(request, env) {
    const path = new URL(request.url).pathname.replace(/\/+$/, "") || "/";

    // comma three gate: catpilot never flashes AGNOS on the comma three
    // (c3_compat targets 12.8 exactly), so refuse the install up front and
    // tell the user how to get there. The setup app shows this text in its
    // download-failed dialog.
    const deviceType = (request.headers.get("x-openpilot-device-type") || "").trim();
    const uaVersion = ((request.headers.get("user-agent") || "").match(/^AGNOSSetup-([\d.]+)/) || [])[1] || "";
    if (deviceType === "tici" && !uaVersion.startsWith("12.8")) {
      return new Response(
        `catpilot on the comma three requires AGNOS 12.8 — this device runs AGNOS ${uaVersion || "unknown"}. ` +
        `First flash AGNOS 12.8 (flash.comma.ai) or install stock openpilot v0.10.0, then install catpilot again.`,
        { status: 409 },
      );
    }

    let branch;
    if (path === "/") {
      const stable = await getAsset(env, "stable");
      branch = stable ? (await stable.text()).trim() : null;
      if (!branch) return new Response("stable pointer not configured", { status: 500 });
    } else if (path === "/dev") {
      branch = "dev";
    } else if (/^\/v\d+\.\d+\.\d+$/.test(path)) {
      branch = path.slice(1);
    } else {
      // 409 bodies are displayed verbatim by the device's setup screen.
      return new Response("Unknown catpilot channel. Use installer.catpilot.dev, /dev, or /vX.Y.Z", { status: 409 });
    }

    if (!(await branchExists(branch))) {
      return new Response(`No catpilot release '${branch}'`, { status: 409 });
    }

    const ua = request.headers.get("user-agent") || "";
    const agnosMajor = parseInt((ua.match(/^AGNOSSetup-(\d+)/) || [])[1] ?? "99", 10);
    const template = agnosMajor < 17 ? "installer-template-legacy" : "installer-template";

    const obj = await getAsset(env, template);
    if (!obj) return new Response("installer template missing", { status: 500 });

    const buf = new Uint8Array(await obj.arrayBuffer());
    const latin1 = new TextDecoder("latin1").decode(buf);
    patchField(buf, latin1, URL_MARKER, URL_FIELD_LEN, GIT_URL);
    patchField(buf, latin1, BRANCH_MARKER, BRANCH_FIELD_LEN, branch);

    return new Response(buf, {
      headers: {
        "content-type": "application/octet-stream",
        "content-length": String(buf.length),
        "cache-control": "no-store",
      },
    });
  },
};
