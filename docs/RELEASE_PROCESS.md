# Release process

Three repos, two cadences:

| Repo | Cadence | Version scheme |
|------|---------|----------------|
| catpilot | **paced by upstream openpilot** — a release per upstream release, cut when our rebase on that version is done | `vX.Y.Z`, mirroring the upstream version |
| plugins | bootstrap at each catpilot release, then **rolling** | bootstrap `vX.Y.Z`, rolling `YYYY.MM.DD` |
| connect-on-device | bootstrap at each catpilot release, then **rolling** | bootstrap `vX.Y.Z`, rolling `YYYY.MM.DD` |

## How versions reach devices

- **catpilot**: `install.catpilot.dev` serves an installer for a release
  branch (`/` → stable pointer, `/vX.Y.Z`, `/dev`). The catpilot repo only
  moves at upstream pace; there are no rolling catpilot updates.
- **plugins**: `first_boot_setup.sh` clones the plugins branch matching the
  installed catpilot branch (`v0.11.1` ⇄ `v0.11.1`). From then on, plugind's
  update checker fetches that same branch — **the release branch is the update
  channel**. A rolling release is simply a push to the channel branch, tagged
  with the date.
- **COD**: always a **pre-built tarball** attached to a GitHub release —
  COD requires a local build step, so devices never clone its repo. The
  channel is encoded in the tag: bootstrap `vX.Y.Z`, rolling
  `vX.Y.Z-YYYY.MM.DD`. First boot picks the newest release for the installed
  channel (rolling first, then bootstrap, then latest as fallback) — so the
  tarball only matters for **new installs**. Devices that already have COD
  update themselves through COD's own self-update.

## Branch & tag conventions

- `dev` — development, both repos. Served as the `/dev` installer channel;
  a device installed from `/dev` tracks `dev` for plugins updates too.
- `release-vX.Y.Z` — one channel **branch** per catpilot release, in catpilot
  and plugins. In catpilot it is frozen at release (fixes only in extremis);
  in plugins it **moves**, carrying rolling releases until the next channel
  opens. The `release-` prefix is not cosmetic: a git tag outranks a branch of
  the same name in ref resolution, so a bare `vX.Y.Z` branch would be shadowed
  by its own tag. Users still type `/v0.11.1` — the installer Worker adds the
  prefix, and `first_boot_setup.sh` strips it back off to get the channel.
- `vX.Y.Z` **tags** — the release point, in all three repos, and what the
  GitHub Releases pages hang off. ⚠ In catpilot this name also exists as an
  upstream commaai tag at a different commit; our tag is pushed to origin
  without creating a local one, so the local upstream reference survives for
  rebase work. Expect `git fetch` to report it as "would clobber existing tag"
  — that rejection is protecting the upstream tag and is safe to ignore.
- `YYYY.MM.DD` tags — mark each rolling release on the plugins channel branch.
  Devices follow the branch; these are bookkeeping.

## Cutting a major release (upstream vX.Y.Z ships)

1. Rebase catpilot on the upstream tag; verify on-car.
2. Push catpilot branch `release-vX.Y.Z`; bump `OPENPILOT_VERSION` in
   `selfdrive/plugins/manifest.py` as part of the rebase.
3. Push plugins branch `release-vX.Y.Z` (from dev, hook-contract-matched to
   the new base); set every `plugin.json` version to `X.Y.Z`.
4. Cut COD GitHub release `vX.Y.Z` with a `cod-vX.Y.Z.tar.gz` asset.
5. Move the installer stable pointer: write `release-vX.Y.Z` into
   `infra/installer/assets/stable`, then `wrangler deploy`. Give it a minute:
   edges serve the old pointer briefly, so never delete the previous branch
   until the root path returns the new one consistently.
6. Tag `vX.Y.Z` in all three repos and create the GitHub Releases.
7. The previous channel branch stops receiving rolling updates.

## Cutting a rolling release (plugins / COD)

1. Land and field-verify the work on `dev`.
2. Merge (or cherry-pick) onto the current channel branch `vX.Y.Z`.
3. Bump `plugin.json` `version` to `YYYY.MM.DD` — **only for plugins that
   changed**. The version field feeds the schema-rebuild hash and the UI, so
   it should reflect when that plugin last changed.
4. Tag `YYYY.MM.DD`, push branch + tag. Devices on that channel pick it up
   through the normal update flow (offroad, `.needs_restart`).
5. For COD: build locally, then publish a GitHub release tagged
   `vX.Y.Z-YYYY.MM.DD` (channel prefix + date) with the `cod-*.tar.gz` asset.

## What may ride a rolling release

Only changes that work against the **frozen catpilot base** of the current
channel — plugin fixes, tuning, new plugins using existing hooks, COD
features. Anything that needs a new hook call site or other catpilot-repo
change waits for the next major release. The architecture enforces this:
plugins can only reach hooks that exist in the installed base.

## Open items

- catpilot's GitHub default branch (`main`) still shows the pre-0.11 lineage;
  repoint or archive it so visitors land on the release line.
- ~~COD self-update should respect the channel~~ — done (COD `2bf0348`):
  the checker lists releases and picks the newest tag on the device's own
  channel (`vX.Y.Z` / `vX.Y.Z-YYYY.MM.DD` sharing VERSION's `X.Y.Z` prefix);
  no release on the channel means no update, never a cross-channel fallback.
