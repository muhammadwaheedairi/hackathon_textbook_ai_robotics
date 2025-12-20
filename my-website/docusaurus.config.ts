import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

// Define environment variables to pass to the client
const siteConfig = {
  // Get environment variables - only pass the ones we want to expose to the client
  RAG_API_URL: process.env.REACT_APP_RAG_API_URL || 'http://localhost:8000',
  NODE_ENV: process.env.NODE_ENV || 'development',
};

const config: Config = {
  title: 'AI-Native Textbook Physical AI & Humanoid Robotics',
  tagline: 'A comprehensive textbook covering ROS 2, Gazebo, NVIDIA Isaac, and Vision-Language-Action systems',
  favicon: 'img/favicon.ico',

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: 'https://muhammadwaheedairi.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/hackathon_textbook_ai_robotics/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'muhammadwaheedairi', // Usually your GitHub org/user name.
  projectName: 'hackathon_textbook_ai_robotics', // Usually your repo name.

  onBrokenLinks: 'throw',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/muhammadwaheedairi/hackathon_textbook_ai_robotics/edit/main/my-website/',
        },
        blog: {
          showReadingTime: true,
          feedOptions: {
            type: ['rss', 'atom'],
            xslt: true,
          },
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/muhammadwaheedairi/hackathon_textbook_ai_robotics/edit/main/my-website/',
          // Useful options to enforce blogging best practices
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
    // Replace with your project's social card
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      defaultMode: 'light',
      disableSwitch: false,
      respectPrefersColorScheme: true,
    },
    // Enhanced accessibility and mobile responsiveness settings
    metadata: [
      {name: 'viewport', content: 'width=device-width, initial-scale=1.0'},
      {name: 'theme-color', content: '#12affa'},
      {name: 'description', content: 'AI-Native Textbook for Physical AI & Humanoid Robotics covering ROS 2, Gazebo, NVIDIA Isaac, and Vision-Language-Action systems'},
      {name: 'keywords', content: 'robotics, AI, textbook, ROS 2, Gazebo, NVIDIA Isaac, humanoid robotics, physical AI, education'},
    ],
    navbar: {
      title: 'AI-Native Textbook',
      logo: {
        alt: 'AI-Native Textbook Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'textbookSidebar',
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
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
    // Custom fields to pass environment variables to the client
    customFields: {
      RAG_API_URL: siteConfig.RAG_API_URL,
      NODE_ENV: siteConfig.NODE_ENV,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
