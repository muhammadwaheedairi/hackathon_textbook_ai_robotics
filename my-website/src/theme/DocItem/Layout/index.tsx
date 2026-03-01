import React, {type ReactNode, useState, useCallback, useRef, useEffect} from 'react';
import clsx from 'clsx';
import {useWindowSize} from '@docusaurus/theme-common';
import {useDoc} from '@docusaurus/plugin-content-docs/client';
import DocItemPaginator from '@theme/DocItem/Paginator';
import DocVersionBanner from '@theme/DocVersionBanner';
import DocVersionBadge from '@theme/DocVersionBadge';
import DocItemFooter from '@theme/DocItem/Footer';
import DocItemTOCMobile from '@theme/DocItem/TOC/Mobile';
import DocItemTOCDesktop from '@theme/DocItem/TOC/Desktop';
import DocItemContent from '@theme/DocItem/Content';
import DocBreadcrumbs from '@theme/DocBreadcrumbs';
import ContentVisibility from '@theme/ContentVisibility';
import type {Props} from '@theme/DocItem/Layout';

import styles from './styles.module.css';

function useDocTOC() {
  const {frontMatter, toc} = useDoc();
  const windowSize = useWindowSize();

  const hidden = frontMatter.hide_table_of_contents;
  const canRender = !hidden && toc.length > 0;

  const mobile = canRender ? <DocItemTOCMobile /> : undefined;
  const desktop =
    canRender && (windowSize === 'desktop' || windowSize === 'ssr') ? (
      <DocItemTOCDesktop />
    ) : undefined;

  return {hidden, mobile, desktop};
}

// ─── Icons ────────────────────────────────────────────────────────────────────

function MarkdownIcon() {
  return (
    <svg width="16" height="16" viewBox="0 0 16 16" fill="none" aria-hidden="true">
      <rect x="1" y="3" width="14" height="10" rx="2" stroke="currentColor" strokeWidth="1.25"/>
      <path d="M3.5 10V6.5l2 2.5 2-2.5V10M10 10V6.5M10 6.5l2 2M10 6.5l-2 2" stroke="currentColor" strokeWidth="1.25" strokeLinecap="round" strokeLinejoin="round"/>
    </svg>
  );
}

function ClaudeIcon() {
  return (
    <svg width="16" height="16" viewBox="0 0 16 16" fill="none" aria-hidden="true">
      <circle cx="8" cy="8" r="6.5" stroke="currentColor" strokeWidth="1.25"/>
      <path d="M5.5 10.5c.5-1.5 1.5-4.5 2.5-4.5s2 3 2.5 4.5" stroke="currentColor" strokeWidth="1.25" strokeLinecap="round"/>
      <path d="M6.5 8.5h3" stroke="currentColor" strokeWidth="1.25" strokeLinecap="round"/>
    </svg>
  );
}

function ChatGPTIcon() {
  return (
    <svg width="16" height="16" viewBox="0 0 16 16" fill="none" aria-hidden="true">
      <circle cx="8" cy="8" r="6.5" stroke="currentColor" strokeWidth="1.25"/>
      <path d="M5 8a3 3 0 0 1 6 0c0 1.5-1 2.5-2 3" stroke="currentColor" strokeWidth="1.25" strokeLinecap="round"/>
      <circle cx="8" cy="5.5" r="0.75" fill="currentColor"/>
    </svg>
  );
}

// ─── Copy Page Button + Dropdown ──────────────────────────────────────────────

function CopyPageButton() {
  const [copied, setCopied] = useState(false);
  const [dropdownOpen, setDropdownOpen] = useState(false);
  const wrapperRef = useRef<HTMLDivElement>(null);

  // Close dropdown when clicking outside
  useEffect(() => {
    function handleClickOutside(e: MouseEvent) {
      if (wrapperRef.current && !wrapperRef.current.contains(e.target as Node)) {
        setDropdownOpen(false);
      }
    }
    document.addEventListener('mousedown', handleClickOutside);
    return () => document.removeEventListener('mousedown', handleClickOutside);
  }, []);

  const handleCopy = useCallback(() => {
    const article = document.querySelector('article');
    if (!article) return;
    const text = (article as HTMLElement).innerText ?? article.textContent ?? '';
    navigator.clipboard.writeText(text).then(() => {
      setCopied(true);
      setTimeout(() => setCopied(false), 2000);
    });
  }, []);

  const handleViewAsMarkdown = useCallback(() => {
    const article = document.querySelector('article');
    if (!article) return;
    const text = (article as HTMLElement).innerText ?? '';
    const blob = new Blob([text], {type: 'text/plain'});
    const url = URL.createObjectURL(blob);
    window.open(url, '_blank');
    setDropdownOpen(false);
  }, []);

  const handleOpenInClaude = useCallback(() => {
    const pageContent = document.querySelector('article');
    const text = pageContent ? (pageContent as HTMLElement).innerText : '';
    const prompt = `Here is a page from the AI-Native Robotics Textbook:\n\n${text.slice(0, 3000)}\n\nPlease help me understand this content.`;
    const url = `https://claude.ai/new?q=${encodeURIComponent(prompt)}`;
    window.open(url, '_blank');
    setDropdownOpen(false);
  }, []);

  const handleOpenInChatGPT = useCallback(() => {
    const pageContent = document.querySelector('article');
    const text = pageContent ? (pageContent as HTMLElement).innerText : '';
    const prompt = `Here is a page from the AI-Native Robotics Textbook:\n\n${text.slice(0, 3000)}\n\nPlease help me understand this content.`;
    const url = `https://chatgpt.com/?q=${encodeURIComponent(prompt)}`;
    window.open(url, '_blank');
    setDropdownOpen(false);
  }, []);

  return (
    <div className={styles.copyPageWrapper} ref={wrapperRef}>
      {/* Main copy button */}
      <button
        onClick={handleCopy}
        className={clsx(styles.copyPageBtn, copied && styles.copyPageBtnCopied)}
        title="Copy page content"
        aria-label="Copy page content">
        {copied ? 'Copied!' : 'Copy page'}
      </button>

      {/* Divider */}
      <div className={styles.copyPageDivider} />

      {/* Chevron toggle */}
      <button
        className={clsx(styles.copyPageChevron, dropdownOpen && styles.copyPageChevronOpen)}
        onClick={() => setDropdownOpen((v) => !v)}
        aria-label="More options"
        aria-expanded={dropdownOpen}>
        <svg width="12" height="12" viewBox="0 0 12 12" fill="none" aria-hidden="true">
          <path d="M2.5 4.5L6 8l3.5-3.5" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round"/>
        </svg>
      </button>

      {/* Dropdown menu */}
      {dropdownOpen && (
        <div className={styles.dropdown}>
          <button className={styles.dropdownItem} onClick={handleViewAsMarkdown}>
            <span className={styles.dropdownIcon}><MarkdownIcon /></span>
            <span className={styles.dropdownText}>
              <span className={styles.dropdownTitle}>View as Markdown</span>
              <span className={styles.dropdownDesc}>Open this page in Markdown</span>
            </span>
          </button>

          <button className={styles.dropdownItem} onClick={handleOpenInClaude}>
            <span className={styles.dropdownIcon}><ClaudeIcon /></span>
            <span className={styles.dropdownText}>
              <span className={styles.dropdownTitle}>Open in Claude</span>
              <span className={styles.dropdownDesc}>Ask questions about this page</span>
            </span>
          </button>

          <button className={styles.dropdownItem} onClick={handleOpenInChatGPT}>
            <span className={styles.dropdownIcon}><ChatGPTIcon /></span>
            <span className={styles.dropdownText}>
              <span className={styles.dropdownTitle}>Open in ChatGPT</span>
              <span className={styles.dropdownDesc}>Ask questions about this page</span>
            </span>
          </button>
        </div>
      )}
    </div>
  );
}

// ─── Layout ───────────────────────────────────────────────────────────────────

export default function DocItemLayout({children}: Props): ReactNode {
  const docTOC = useDocTOC();
  const {metadata} = useDoc();

  return (
    <div className="row">
      <div className={clsx('col', !docTOC.hidden && styles.docItemCol)}>
        <ContentVisibility metadata={metadata} />
        <DocVersionBanner />
        <div className={styles.docItemContainer}>
          <article>
            <div className={styles.docTopBar}>
              <div className={styles.breadcrumbsWrapper}>
                <DocBreadcrumbs />
              </div>
              <CopyPageButton />
            </div>
            <DocVersionBadge />
            {docTOC.mobile}
            <DocItemContent>{children}</DocItemContent>
            <DocItemFooter />
          </article>
          <DocItemPaginator />
        </div>
      </div>
      {docTOC.desktop && <div className="col col--3">{docTOC.desktop}</div>}
    </div>
  );
}