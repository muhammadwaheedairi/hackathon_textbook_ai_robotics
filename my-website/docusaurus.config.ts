import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

// Define environment variables to pass to the client
const siteConfig = {
  RAG_API_URL: process.env.REACT_APP_RAG_API_URL ||
    (process.env.NODE_ENV === 'production'
      ? 'https://muhammadwaheedairi-rag-chatbot-textbook.hf.space'
      : 'http://localhost:8000'),
  NODE_ENV: process.env.NODE_ENV || 'development',
};

const config: Config = {
  title: 'AI-Native Textbook Physical AI & Humanoid Robotics',
  tagline: 'A comprehensive textbook covering ROS 2, Gazebo, NVIDIA Isaac, and Vision-Language-Action systems',
  favicon: 'img/favicon.ico',


  url: 'https://muhammadwaheedairi.github.io',
  baseUrl: '/hackathon_textbook_ai_robotics/',

  organizationName: 'muhammadwaheedairi',
  projectName: 'hackathon_textbook_ai_robotics',

  onBrokenLinks: 'throw',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  plugins: [
    [
      require.resolve('@easyops-cn/docusaurus-search-local'),
      {
        hashed: true,
        language: ['en'],
        indexDocs: true,
        indexBlog: false,
        indexPages: false,
        docsRouteBasePath: '/docs',
        searchBarPosition: 'right',
        searchBarShortcutHint: true,
        searchResultLimits: 8,
        searchResultContextMaxLength: 50,
        highlightSearchTermsOnTargetPage: true,
        explicitSearchResultPath: true,
      },
    ],
  ],

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          editUrl:
            'https://github.com/muhammadwaheedairi/hackathon_textbook_ai_robotics/edit/main/my-website/',
        },
        blog: {
          showReadingTime: true,
          feedOptions: {
            type: ['rss', 'atom'],
            xslt: true,
          },
          editUrl:
            'https://github.com/muhammadwaheedairi/hackathon_textbook_ai_robotics/edit/main/my-website/',
          onInlineTags: 'warn',
          onInlineAuthors: 'warn',
          onUntruncatedBlogPosts: 'warn',
        },
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    image: 'img/docusaurus-social-card.jpg',

    // ✅ Default dark mode
    colorMode: {
      defaultMode: 'dark',
      disableSwitch: false,
      respectPrefersColorScheme: true,
    },

    metadata: [
      {name: 'viewport', content: 'width=device-width, initial-scale=1.0'},
      {name: 'theme-color', content: '#0070f3'},
      {name: 'description', content: 'AI-Native Textbook for Physical AI & Humanoid Robotics covering ROS 2, Gazebo, NVIDIA Isaac, and Vision-Language-Action systems'},
      {name: 'keywords', content: 'robotics, AI, textbook, ROS 2, Gazebo, NVIDIA Isaac, humanoid robotics, physical AI, education'},
    ],

    navbar: {
      title: 'AI-Native Textbook',
      hideOnScroll: true,
      logo: {
        alt: 'AI-Native Textbook Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Textbook',
        },
        {to: '/blog', label: 'Blog', position: 'left'},
        {
          href: 'https://github.com/muhammadwaheedairi/hackathon_textbook_ai_robotics',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },

    footer: {
      style: 'dark',
      links: [
        {
          title: 'Textbook',
          items: [
            {
              label: 'Introduction',
              to: '/docs/intro',
            },
          ],
        },
        {
          title: 'Resources',
          items: [
            {
              label: 'ROS 2 Documentation',
              href: 'https://docs.ros.org/en/humble/',
            },
            {
              label: 'NVIDIA Isaac',
              href: 'https://developer.nvidia.com/isaac',
            },
            {
              label: 'Gazebo Simulation',
              href: 'https://gazebosim.org/',
            },
          ],
        },
        {
          title: 'More',
          items: [
            {
              label: 'Blog',
              to: '/blog',
            },
            {
              label: 'GitHub',
              href: 'https://github.com/muhammadwaheedairi/hackathon_textbook_ai_robotics',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} AI-Native Textbook — Physical AI & Humanoid Robotics. Built with Docusaurus.`,
    },

    // ✅ VS Dark theme + additional languages
    prism: {
      theme: prismThemes.vsDark,
      darkTheme: prismThemes.vsDark,
      additionalLanguages: ['bash', 'python', 'yaml', 'cpp', 'csharp', 'typescript'],
    },

    customFields: {
      RAG_API_URL: siteConfig.RAG_API_URL,
      NODE_ENV: siteConfig.NODE_ENV,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;