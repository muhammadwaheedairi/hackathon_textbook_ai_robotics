# ADR-0002: Search Implementation Strategy

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-16
- **Feature:** 001-textbook-search
- **Context:** Need to implement a search functionality for textbook content pages that highlights matching keywords, supports partial matches, and uses debounced real-time updates. The solution must work within the existing Docusaurus-based textbook platform without requiring backend infrastructure.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

- **Search Algorithm**: Client-side search using FlexSearch library for optimal performance with static content
- **Debouncing**: 300ms debounce delay after user stops typing to optimize performance
- **Highlighting**: DOM manipulation to wrap matching text in highlighted spans with CSS styling
- **Partial Matching**: Regex-based substring matching to support finding content even with partial terms
- **Architecture**: React components integrated into Docusaurus theme for seamless integration

<!-- For technology stacks, list all components:
     - Framework: Next.js 14 (App Router)
     - Styling: Tailwind CSS v3
     - Deployment: Vercel
     - State Management: React Context (start simple)
-->

## Consequences

### Positive

- Fast, responsive search experience without server round-trips
- No external dependencies or API calls required
- Works offline with static content
- Maintains existing textbook structure without modifications
- Good performance with large content sets due to optimized search library
- Accessible and keyboard navigable search interface

<!-- Example: Integrated tooling, excellent DX, fast deploys, strong TypeScript support -->

### Negative

- Initial content indexing may increase bundle size
- Memory usage increases with content size
- Search index needs to be rebuilt when content changes
- Limited advanced search features compared to backend solutions
- Potential performance issues with very large content sets if not properly optimized

<!-- Example: Vendor lock-in to Vercel, framework coupling, learning curve -->

## Alternatives Considered

- **Full-text search libraries (FlexSearch, MiniSearch)**: Selected approach - good performance, client-side, minimal bundle impact
- **Simple regex-based search**: Rejected - poor performance with large content sets, limited features
- **External search service (Algolia, Google CSE)**: Rejected - external dependency, potential costs, privacy concerns
- **Static-site generated search index**: Rejected - more complex build process, requires build-time generation
- **Native browser search (Ctrl+F)**: Rejected - doesn't provide integrated UI or highlighting within page content

<!-- Group alternatives by cluster:
     Alternative Stack A: Remix + styled-components + Cloudflare
     Alternative Stack B: Vite + vanilla CSS + AWS Amplify
     Why rejected: Less integrated, more setup complexity
-->

## References

- Feature Spec: specs/001-textbook-search/spec.md
- Implementation Plan: specs/001-textbook-search/plan.md
- Related ADRs: null
- Evaluator Evidence: specs/001-textbook-search/research.md <!-- link to eval notes/PHR showing graders and outcomes -->
