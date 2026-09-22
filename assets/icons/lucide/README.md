# Vendored Lucide icons

These SVGs come from `lucide-static` (currently pinned to `1.31.0` in [`package.json`](../../../package.json), with the
resolved package in [`package-lock.json`](../../../package-lock.json)).
The complete upstream license is copied to [LICENSE](LICENSE), including the MIT notice for Feather-derived icons.
Upstream license comments are retained in each SVG.

Regenerate from the repository root:

```sh
npm ci
node tools/vendor-lucide.mjs
```

The [vendoring script](../../../tools/vendor-lucide.mjs) selects the six icons used by the control tabs and replaces
`stroke="currentColor"` with `stroke="white"`.
This lets egui tint the rasterized SVGs with the normal or selected foreground color.
No other SVG changes are made.
The files are committed and embedded by Rust, so native builds do not require Node or npm.

To update Lucide, update the pinned npm dependency and lockfile, rerun the script, and review the SVG and license
changes.
To change the selection, edit the script's `icons` list and the corresponding Rust tab mapping; remove unused vendored
SVGs manually.
The script overwrites selected SVGs and the license, but deletes nothing.

[`web/lucide-icons.mjs`](../../../web/lucide-icons.mjs) separately copies the web shell's icons into Trunk's staging
directory without this color adjustment.
