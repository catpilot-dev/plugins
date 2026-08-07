# installer.catpilot.dev

Serves the catpilot installer binary to comma devices, replacing
`installer.comma.ai/<user>/<branch>`. A user types one of these into
**Custom Software** during device setup:

| URL | Installs |
|-----|----------|
| `installer.catpilot.dev` | latest stable release |
| `installer.catpilot.dev/dev` | dev branch |
| `installer.catpilot.dev/v0.11.1` | that specific release |

## How it works

The device setup app passes any dotted URL through unmodified and requires the
response to be an aarch64 ELF, which it saves to `/tmp/installer` and executes
(see catpilot `system/ui/tici_setup.py`). The installer binary embeds a
`?`-terminated, space-padded git URL and branch — a format comma designed for
fork installers. The Worker holds one padded template binary in R2 and rewrites
those two fields per request; the branch comes from the URL path (`/` resolves
via the `stable` pointer object in R2).

Error responses use HTTP 409 because the setup screen displays a 409 body
verbatim to the user.

## Deploy (once)

```bash
cd infra/installer
wrangler login
wrangler r2 bucket create catpilot-installers

# templates: comma's padded installers, fetched with device user-agents.
# One build covers tici/tizi/mici; the AGNOS generation picks the build
# (worker serves -legacy to AGNOSSetup major < 17, i.e. the comma three on 12.8).
curl -s -A "AGNOSSetup-19.4" -o installer-template \
  "https://installer.comma.ai/commaai/release3"
curl -s -A "AGNOSSetup-12.8" -o installer-template-legacy \
  "https://installer.comma.ai/commaai/release3"
file installer-template installer-template-legacy  # both: ELF 64-bit LSB, ARM aarch64
wrangler r2 object put catpilot-installers/installer-template --file installer-template
wrangler r2 object put catpilot-installers/installer-template-legacy --file installer-template-legacy

# stable pointer
printf 'v0.11.1' > stable.txt
wrangler r2 object put catpilot-installers/stable --file stable.txt

wrangler deploy          # creates the custom domain + DNS + cert automatically
```

Smoke test (any machine):

```bash
curl -s https://installer.catpilot.dev/v0.11.1 -o /tmp/inst
file /tmp/inst                                  # aarch64 ELF
strings -a /tmp/inst | grep catpilot.git        # patched URL
strings -a /tmp/inst | grep '^v0\.11\.1?'       # patched branch
curl -s -o /dev/null -w '%{http_code}\n' https://installer.catpilot.dev/v9.9.9   # 409
```

## Per release

1. Push the release branch `vX.Y.Z` in the catpilot repo (and the matching
   branch in the plugins repo; cut the COD GitHub release).
2. Move the stable pointer:
   ```bash
   printf 'vX.Y.Z' > stable.txt
   wrangler r2 object put catpilot-installers/stable --file stable.txt
   ```
   No Worker redeploy needed.

## Template maintenance

The templates are comma's own builds, so they match the AGNOS libraries of the
devices that request them: `installer-template` is the current-AGNOS build
(3X / comma four), `installer-template-legacy` the old-AGNOS build that comma
still serves to AGNOS 12.8 (comma three). Refresh both (same `curl` as above)
whenever AGNOS majors bump. Empirical facts these choices rest on (2026-08-08):
tici/tizi/mici receive the same BuildID at a given AGNOS version — only the
baked branch differs — while AGNOS 12.8 receives a distinct older build. The
comma-three fresh-install path still has its own open question (which AGNOS a
factory-reset C3 runs, and c3_compat's 12.8 assumption).
