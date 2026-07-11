# Contributing to sw2robot

Thanks for your interest in improving sw2robot. This document explains how to
propose changes so that they are reviewable by a small maintainer team.

Please read the two short sections below before opening an issue or a pull
request. They exist to keep review load sustainable — not to gatekeep.

## TL;DR

- **Discuss first, code second.** Open an issue and get a maintainer to agree on
  the approach before sending an implementation PR.
- **One PR = one reviewable change.** Small and focused beats large and stacked.
- **Disclose AI usage.** Say which tool you used and for what. See below.
- **Verify before you send.** If you didn't run it, it isn't ready.

## Issues

- **Keep it short and specific.** State the problem, what you expected, and (for
  bugs) exact steps to reproduce. A few focused sentences are worth more than a
  long essay.
- **One topic per issue.** Don't batch several unrelated discussions into one
  thread, and don't open many broad "discussion" issues at once — maintainers
  cannot review them in parallel. Land one before opening the next.
- **Human-in-the-loop for AI.** You may use AI to help draft an issue, but a
  human must read, edit, and stand behind every word before you post it.
  Unedited, machine-generated walls of text will be closed with a request to
  condense.

### Attach a reproducible sample

This is an open-source project, so anything a maintainer cannot reproduce is
very hard to act on. If your issue involves a specific SolidWorks assembly or
part — a bug during extract/build, a wrong result, a shape the tool mishandles —
please attach the relevant `.sldasm` / `.sldprt` files (and any references they
need) as a **`.zip`** so we can open and inspect them here.

Two rules keep this practical and safe:

- **Make it minimal.** Include only what is needed to reproduce the behavior —
  the smallest assembly/parts that still show the problem, not your whole
  production model.
- **Never upload confidential CAD.** If the real parts are proprietary, replace
  them with **dummy stand-ins** — simple placeholder geometry that reproduces
  the same behavior (the mate structure, subassembly nesting, coordinate frame,
  or shape characteristic that matters). Share the reproduction, not your
  secrets.

If you genuinely cannot share a reproducible sample, say so and describe the
structure precisely instead — but expect that undiagnosable reports may be
closed.

## Pull requests

### Issue-first

Implementation PRs must reference an **accepted** issue — an issue a maintainer
has labelled [`accepted`](https://github.com/jsk-ros-pkg/solidworks_urdf_exporter2/issues?q=is%3Aissue+label%3Aaccepted),
meaning the direction has been agreed. Open an issue, discuss it, and wait for
that label before writing the implementation. Drive-by PRs that do not reference
an `accepted` issue may be closed without a full review, regardless of how much
code they contain. This is the single most important rule here: it prevents
wasted work on both sides.

**Maintainers are exempt.** Maintainers may open PRs directly without a
pre-filed `accepted` issue — they own the roadmap and are trusted to apply good
judgment. The keep-it-small, verify-it, and demo-for-UI rules below still apply
to everyone, maintainers included. For a non-trivial or contentious change, a
maintainer is still encouraged to open an issue first so others can weigh in.

### Keep PRs small and independent

- Aim for one logical change per PR. If you find yourself writing "Part 3 of 7"
  or stacking many branches, that is a signal to stop and talk to a maintainer
  about scope first.
- Large stacked PRs (thousands of lines across many dependent branches) will be
  sent back for splitting. Reviewers review the **diff**, not the description —
  a change that cannot be understood from its diff is not ready.
- Write the PR description for a human reviewer: a few lines on *what* changed
  and *why*. Skip auto-generated multi-section essays.

### Verify your change

Before requesting review, make sure:

- The code builds and the test suite passes locally.
- You have **actually run** the change and confirmed it does what the PR claims,
  on the platforms it affects (note that `extract` is Windows + SolidWorks only).
- New behavior has a test where practical.

### Show your work for UI changes

If your PR changes the browser editor UI or any user-visible behavior, include
**proof that you ran it**:

- A **short screen recording** (a few seconds is enough) of the new behavior in
  action is required for interactive changes — a new control, a changed flow, a
  fixed visual bug. GitHub lets you drag a video straight into the PR
  description.
- A **before/after screenshot** is acceptable for a purely static visual change
  (layout, color, label).

This is not busywork: a working demo is the clearest evidence that the change
does what the description says. PRs that alter the UI without a demo will be sent
back with a request for one before review.

## AI usage policy

sw2robot is not anti-AI. Maintainers use AI tooling too. We care about the
**quality** of a contribution, not how it was produced. The rules exist because
low-effort, unverified AI output shifts the entire cost of a contribution onto
the people reviewing it.

1. **Disclose all AI usage.** In the PR/issue, state the tool(s) used and the
   extent of assistance (e.g. "drafted with an LLM, reviewed and edited by
   hand", or "wrote the tests myself, used AI for the docstrings").
2. **AI-assisted implementation PRs are for `accepted` issues only.** See
   Issue-first above.
3. **You are responsible for everything you submit.** "The AI wrote it" is not
   an explanation for a bug or a design choice. You must understand and be able
   to defend every line.
4. **No AI-generated media** (images, video, audio) in issues, PRs, or the repo.
5. **Repeatedly submitting unverified AI slop** — fabricated bug reports, code
   the author clearly doesn't understand, or floods of stacked PRs/issues — may
   result in being blocked from further contributions.

If you follow these rules, AI assistance is welcome. If you use it to generate
volume you haven't reviewed, expect it to be closed.

## Questions

If you're unsure whether something is in scope or how big a change should be,
open an issue and ask before writing code. That conversation is always cheaper
than a rejected PR.
