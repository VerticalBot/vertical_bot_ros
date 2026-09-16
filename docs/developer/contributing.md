# Contributing

- Keep the RoboDK API surface compatible: names, argument order and return types follow the official Python
  package (`robodk.robolink`). New studio-only features get new methods rather than changed semantics.
- Units: mm and degrees everywhere in the item model; convert at the boundaries (URDF, glTF, VDA 5050).
- Every feature ships with a vitest test and a documentation page or section; the {doc}`../roadmap` is
  updated in the same change.
- Post processors: add a TypeScript post under `src/posts/` with a golden-file test, or document how to use the
  vendor's RoboDK post.
- Run `npm run typecheck && npm test && npm run build` before pushing; the CI runs the same.
- Commit messages describe the user-visible change first, then the internals.
