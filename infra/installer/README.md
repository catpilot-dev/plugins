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

# template: comma's padded installer, fetched with a device user-agent
curl -s -A "AGNOSSetup-12.8" -o installer-template \
  "https://installer.comma.ai/commaai/release3"
file installer-template   # must say: ELF 64-bit LSB executable, ARM aarch64
wrangler r2 object put catpilot-installers/installer-template --file installer-template

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

The template is comma's own build, so it matches current AGNOS libraries.
Refresh it (same `curl` as above) whenever AGNOS majors bump. Older AGNOS
(e.g. the comma three's 12.8) may predate the glibc this binary links against —
the comma-three fresh-install path has its own open questions (AGNOS version,
c3_compat) and is not covered by this service yet.
