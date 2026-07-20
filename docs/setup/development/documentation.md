# Working on the documentation

These docs are a static website built with [MkDocs](https://www.mkdocs.org/)
using the [Material for MkDocs](https://squidfunk.github.io/mkdocs-material/)
theme. You write pages in plain **Markdown** (`.md`) and MkDocs turns them into
the site you're reading. This page explains how to preview, build, and publish
them.

Everything runs through the same `make` + `uv` model as the rest of the project,
so you don't install MkDocs system-wide — `uv` puts it in an isolated
environment for you.

---

## Preview locally (live reload)

From the repo root:

```bash
make docs-serve
```

This runs `mkdocs serve` in the docs environment and starts a local web server.
Open the URL it prints — usually **<http://localhost:8000>** — in your browser.
As you edit and save any `.md` file, the page reloads automatically. Press
`Ctrl`+`C` in the terminal to stop the server.

!!! tip "First run"
    `make docs-serve` runs `make sync-docs` first (`uv sync --group docs`), which
    creates the docs environment with MkDocs, the Material theme, and the
    plugins. This only happens once; later runs start immediately.

---

## Build and deploy

Build the static site into the `site/` folder (what you'd upload to a web host):

```bash
make docs        # = uv run mkdocs build
```

Publish it to GitHub Pages:

```bash
make docs-deploy # = uv run mkdocs gh-deploy --force
```

`gh-deploy` builds the site and pushes it to the `gh-pages` branch, which GitHub
serves at the project's Pages URL. You need push access to the repo.

!!! warning "`--force`"
    `make docs-deploy` uses `--force`, which overwrites the published site with
    your current build. Make sure your local docs are correct (preview them
    first) before deploying.

---

## Where the pages live

- **`docs/`** — all the Markdown pages (this page is
  `docs/setup/development/documentation.md`). Folders here map to sections of the
  site.
- **`mkdocs.yml`** (repo root) — the site config: theme, plugins, Markdown
  extensions, and the **`nav`** tree. **If you add a new page, add it to the
  `nav:` section of `mkdocs.yml`** or it won't appear in the navigation.
- **`docs/main.py`** — the macros module (see below).
- **`docs/stylesheets/`** — extra CSS.

Internal links are **relative paths to the `.md` file**, and MkDocs checks them.
For example, from a page in `docs/setup/` you link to the architecture page (at
the docs root) as `../architecture.md`, and to a sibling as `client.md`.

---

## Formatting conventions

The Material theme is enabled with a set of Markdown extensions (configured in
`mkdocs.yml`). The ones you'll use most:

**Admonitions** — the coloured callout boxes. Write `!!!` then a type and a
title, and indent the body by four spaces:

```markdown
!!! note "Optional title"
    Body text, indented four spaces.
```

Common types: `note`, `tip`, `warning`, `danger`, `failure`. Use `???` instead of
`!!!` for a **collapsible** box (starts closed):

```markdown
??? note "Click to expand"
    Hidden until the reader clicks.
```

**Code fences** — wrap commands and code in triple backticks with a language for
syntax highlighting and a copy button (the theme adds it automatically):

````markdown
```bash
make docs-serve
```
````

**Tables, footnotes, definition lists, and inline highlighting** are all
available (via the `tables`, `footnotes`, `def_list`, and `pymdownx` extensions).
See the [Material reference](https://squidfunk.github.io/mkdocs-material/reference/)
for the full palette.

---

## The macros plugin

The site uses the [mkdocs-macros](https://mkdocs-macros-plugin.readthedocs.io/)
plugin, pointed at **`docs/main.py`** (`module_name: docs/main` in `mkdocs.yml`).
Macros are Python functions you can call from inside a page with
`{% raw %}{{ ... }}{% endraw %}` syntax.

Currently `docs/main.py` defines one macro, `lorem(n)`, which generates `n`
paragraphs of placeholder text — handy while drafting. Call it like
`{% raw %}{{ lorem(2) }}{% endraw %}`.

!!! warning "Remove placeholder macros before publishing"
    `{% raw %}{{ lorem(...) }}{% endraw %}` is scaffolding for drafts. Don't
    leave it in a finished page — replace it with real content.
