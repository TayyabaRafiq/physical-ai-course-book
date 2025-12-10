// @ts-check

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'VLA Integration',
  tagline: 'Vision-Language-Action Pipeline for Robot Control',
  favicon: 'img/favicon.ico',

  url: 'https://your-docusaurus-site.example.com',
  baseUrl: '/',

  organizationName: 'vla-team',
  projectName: 'vla-integration',

  onBrokenLinks: 'warn',
  onBrokenMarkdownLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: './sidebars.js',
          editUrl:
            'https://github.com/your-org/vla-integration/tree/main/docs/',
        },
        blog: false,
        theme: {
          customCss: './src/css/custom.css',
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      image: 'img/vla-social-card.jpg',
      navbar: {
        title: 'VLA Integration',
        logo: {
          alt: 'VLA Logo',
          src: 'img/logo.svg',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Documentation',
          },
          {
            href: 'https://github.com/your-org/vla-integration',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Docs',
            items: [
              {
                label: 'Introduction',
                to: '/docs/intro',
              },
              {
                label: 'Architecture',
                to: '/docs/architecture/overview',
              },
              {
                label: 'API Reference',
                to: '/docs/api/overview',
              },
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/your-org/vla-integration',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} VLA Integration Team.`,
      },
      prism: {
        theme: require('prism-react-renderer').themes.github,
        darkTheme: require('prism-react-renderer').themes.dracula,
        additionalLanguages: ['python', 'bash', 'json'],
      },
    }),

  markdown: {
    mermaid: true,
  },

  themes: ['@docusaurus/theme-mermaid'],
};

module.exports = config;