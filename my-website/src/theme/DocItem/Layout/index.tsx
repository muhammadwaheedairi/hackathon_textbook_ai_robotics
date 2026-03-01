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

function UrduIcon() {
  return (
    <svg width="16" height="16" viewBox="0 0 24 24" fill="none" aria-hidden="true">
      <path d="M12.87 15.07l-2.54-2.51.03-.03A17.52 17.52 0 0014.07 6H17V4h-7V2H8v2H1v2h11.17C11.5 7.92 10.44 9.75 9 11.35 8.07 10.32 7.3 9.19 6.69 8h-2c.73 1.63 1.73 3.17 2.98 4.56l-5.09 5.02L4 19l5-5 3.11 3.11.76-2.04zM18.5 10h-2L12 22h2l1.12-3h4.75L21 22h2l-4.5-12zm-2.62 7l1.62-4.33L19.12 17h-3.24z" fill="currentColor"/>
    </svg>
  );
}

// ─── Translate Button ─────────────────────────────────────────────────────────

function TranslateButton() {
  const [isTranslated, setIsTranslated] = useState(false);
  const [isLoading, setIsLoading] = useState(false);
  const originalContentRef = useRef<string | null>(null);

  const translateText = async (text: string): Promise<string> => {
    const chunks = text.match(/.{1,500}/gs) || [];
    const translated: string[] = [];

    for (const chunk of chunks) {
      const res = await fetch(
        `https://api.mymemory.translated.net/get?q=${encodeURIComponent(chunk)}&langpair=en|ur`
      );
      const data = await res.json();
      translated.push(data.responseData?.translatedText || chunk);
    }

    return translated.join(' ');
  };

  const handleTranslate = useCallback(async () => {
    const article = document.querySelector('article');
    if (!article) return;

    if (isTranslated) {
      // Restore original
      if (originalContentRef.current) {
        article.innerHTML = originalContentRef.current;
      }
      setIsTranslated(false);
      return;
    }

    // Save original
    originalContentRef.current = article.innerHTML;
    setIsLoading(true);

    try {
      // Get all text nodes to translate
      const walker = document.createTreeWalker(
        article,
        NodeFilter.SHOW_TEXT,
        {
          acceptNode: (node) => {
            const parent = node.parentElement;
            if (!parent) return NodeFilter.FILTER_REJECT;
            const tag = parent.tagName.toLowerCase();
            if (['script', 'style', 'code', 'pre'].includes(tag)) return NodeFilter.FILTER_REJECT;
            if (node.textContent?.trim() === '') return NodeFilter.FILTER_REJECT;
            return NodeFilter.FILTER_ACCEPT;
          }
        }
      );

      const textNodes: Text[] = [];
      let node;
      while ((node = walker.nextNode())) {
        textNodes.push(node as Text);
      }

      // Translate in batches
      for (const textNode of textNodes) {
        const original = textNode.textContent?.trim();
        if (!original || original.length < 3) continue;
        const res = await fetch(
          `https://api.mymemory.translated.net/get?q=${encodeURIComponent(original)}&langpair=en|ur`
        );
        const data = await res.json();
        const translated = data.responseData?.translatedText;
        if (translated && translated !== original) {
          textNode.textContent = translated;
        }
      }

      setIsTranslated(true);
    } catch (err) {
      console.error('Translation failed:', err);
      // Restore on error
      if (originalContentRef.current) {
        article.innerHTML = originalContentRef.current;
      }
    } finally {
      setIsLoading(false);
    }
  }, [isTranslated]);

  return (
    <button
      onClick={handleTranslate}
      disabled={isLoading}
      title={isTranslated ? 'Show original English' : 'Translate to Urdu'}
      aria-label={isTranslated ? 'Show original English' : 'Translate to Urdu'}
      style={{
        display: 'inline-flex',
        alignItems: 'center',
        gap: '6px',
        padding: '0 0.875rem',
        height: '32px',
        fontSize: '0.8rem',
        fontWeight: 500,
        fontFamily: 'var(--ifm-font-family-base)',
        color: isTranslated ? '#ffffff' : '#374151',
        background: isTranslated ? '#0070f3' : '#ffffff',
        border: '1px solid',
        borderColor: isTranslated ? '#0070f3' : '#e5e7eb',
        borderRadius: '6px',
        cursor: isLoading ? 'wait' : 'pointer',
        whiteSpace: 'nowrap',
        transition: 'all 0.15s ease',
        marginLeft: '8px',
      }}>
      {isLoading ? (
        <>
          <svg width="14" height="14" viewBox="0 0 24 24" fill="none" aria-hidden="true"
            style={{animation: 'spin 1s linear infinite'}}>
            <circle cx="12" cy="12" r="10" stroke="currentColor" strokeWidth="3" strokeDasharray="31.4" strokeDashoffset="10"/>
          </svg>
          ترجمہ ہو رہا ہے...
        </>
      ) : isTranslated ? (
        <>
          <UrduIcon />
          Show English
        </>
      ) : (
        <>
          <UrduIcon />
          اردو
        </>
      )}
    </button>
  );
}

// ─── Copy Page Button + Dropdown ──────────────────────────────────────────────

function CopyPageButton() {
  const [copied, setCopied] = useState(false);
  const [dropdownOpen, setDropdownOpen] = useState(false);
  const wrapperRef = useRef<HTMLDivElement>(null);

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
      <button
        onClick={handleCopy}
        className={clsx(styles.copyPageBtn, copied && styles.copyPageBtnCopied)}
        title="Copy page content"
        aria-label="Copy page content">
        {copied ? 'Copied!' : 'Copy page'}
      </button>

      <div className={styles.copyPageDivider} />

      <button
        className={clsx(styles.copyPageChevron, dropdownOpen && styles.copyPageChevronOpen)}
        onClick={() => setDropdownOpen((v) => !v)}
        aria-label="More options"
        aria-expanded={dropdownOpen}>
        <svg width="12" height="12" viewBox="0 0 12 12" fill="none" aria-hidden="true">
          <path d="M2.5 4.5L6 8l3.5-3.5" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round"/>
        </svg>
      </button>

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
              <div style={{display:'flex', alignItems:'center'}}>
                <CopyPageButton />
                <TranslateButton />
              </div>
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