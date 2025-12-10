/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation
 */

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  tutorialSidebar: [
    {
      type: 'doc',
      id: 'intro',
      label: 'Introduction',
    },
    {
      type: 'category',
      label: 'Architecture',
      items: [
        'architecture/overview',
        'architecture/voice-layer',
        'architecture/cognition-layer',
        'architecture/execution-layer',
        'architecture/sequence-flow',
      ],
    },
    {
      type: 'category',
      label: 'Tutorials',
      items: [
        'tutorials/installation',
        'tutorials/first-command',
        'tutorials/multi-step-task',
        'tutorials/troubleshooting',
      ],
    },
    {
      type: 'category',
      label: 'API Reference',
      items: [
        'api/overview',
        'api/data-models',
        'api/interfaces',
        'api/ros-actions',
      ],
    },
    {
      type: 'doc',
      id: 'developer-guide',
      label: 'Developer Guide',
    },
  ],
};

module.exports = sidebars;