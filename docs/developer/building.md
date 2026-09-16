# Building and testing

```bash
cd studio
npm install
npm run dev            # Vite dev server with hot reload
npm run typecheck      # TypeScript
npm test               # vitest unit tests (STUDIO_NET_TESTS=1 also runs the online-library downloads)
npm run build          # production bundle in dist/
npm run build:server && node dist-server/index.js   # or: npm run server
npm run smoke          # Playwright smoke run over the demo stations (needs the preview server on :4173)
```

Python parts: `python3 studio/blender/test_core.py <station> <fk.json>` is driven by vitest;
`studio/python/examples/*.py` run against a started server.

## Documentation

This documentation lives in `docs/` (Sphinx + MyST Markdown) and is built by Read the Docs from the
repository (`.readthedocs.yaml`). Build locally:

```bash
pip install -r docs/requirements.txt
sphinx-build -b html docs docs/_build/html
```

Pages under `studio/docs/*.md` are included verbatim so they stay next to the code; new chapters go to
`docs/<section>/<page>.md` and into the toctree in `docs/index.md`.

## Continuous integration

`.github/workflows/studio.yml` runs typecheck, tests and the build; `.github/workflows/docs.yml` builds the
documentation on every push and pull request.
