import type { ReactNode } from 'react';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import styles from './index.module.css';

// ─── Hero ─────────────────────────────────────────────────────────────────────

function Hero() {
  return (
    <section className={styles.hero}>
      <div className={styles.heroInner}>
        <div className={styles.heroBadge}>13-Week Curriculum · 26 Chapters</div>
        <h1 className={styles.heroTitle}>
          Physical AI &{' '}
          <span className={styles.heroAccent}>Humanoid Robotics</span>
        </h1>
        <p className={styles.heroSubtitle}>
          A comprehensive AI-native textbook covering ROS 2, Gazebo, NVIDIA Isaac,
          and Vision-Language-Action systems — from fundamentals to autonomous
          humanoid deployment.
        </p>
        <div className={styles.heroButtons}>
          <Link className={styles.btnPrimary} to="/docs/intro">
            Start Learning
            <svg width="16" height="16" viewBox="0 0 16 16" fill="none">
              <path d="M3 8h10M9 4l4 4-4 4" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round" />
            </svg>
          </Link>
          <Link
            className={styles.btnSecondary}
            to="https://github.com/muhammadwaheedairi/hackathon_textbook_ai_robotics">
            <svg width="16" height="16" viewBox="0 0 24 24" fill="currentColor">
              <path d="M12 2C6.477 2 2 6.484 2 12.017c0 4.425 2.865 8.18 6.839 9.504.5.092.682-.217.682-.483 0-.237-.008-.868-.013-1.703-2.782.605-3.369-1.343-3.369-1.343-.454-1.158-1.11-1.466-1.11-1.466-.908-.62.069-.608.069-.608 1.003.07 1.531 1.032 1.531 1.032.892 1.53 2.341 1.088 2.91.832.092-.647.35-1.088.636-1.338-2.22-.253-4.555-1.113-4.555-4.951 0-1.093.39-1.988 1.029-2.688-.103-.253-.446-1.272.098-2.65 0 0 .84-.27 2.75 1.026A9.564 9.564 0 0 1 12 6.844a9.59 9.59 0 0 1 2.504.337c1.909-1.296 2.747-1.027 2.747-1.027.546 1.379.202 2.398.1 2.651.64.7 1.028 1.595 1.028 2.688 0 3.848-2.339 4.695-4.566 4.943.359.309.678.92.678 1.855 0 1.338-.012 2.419-.012 2.747 0 .268.18.58.688.482A10.02 10.02 0 0 0 22 12.017C22 6.484 17.522 2 12 2z" />
            </svg>
            View on GitHub
          </Link>
        </div>

        <div className={styles.heroStats}>
          {[
            { number: '4', label: 'Modules' },
            { number: '13', label: 'Weeks' },
            { number: '26', label: 'Chapters' },
            { number: '100+', label: 'Code Examples' },
          ].map((stat, i, arr) => (
            <div key={stat.label} className={styles.statGroup}>
              <div className={styles.stat}>
                <span className={styles.statNumber}>{stat.number}</span>
                <span className={styles.statLabel}>{stat.label}</span>
              </div>
              {i < arr.length - 1 && <div className={styles.statDivider} />}
            </div>
          ))}
        </div>
      </div>

      {/* Decorative dots grid */}
      <div className={styles.heroGrid} aria-hidden="true" />
    </section>
  );
}

// ─── Tech Strip ───────────────────────────────────────────────────────────────

const techStack = [
  { label: 'ROS 2', color: '#3b82f6', bg: '#eff6ff' },
  { label: 'Gazebo', color: '#6366f1', bg: '#eef2ff' },
  { label: 'NVIDIA Isaac', color: '#10b981', bg: '#ecfdf5' },
  { label: 'Unity HDRP', color: '#f59e0b', bg: '#fffbeb' },
  { label: 'OpenAI Whisper', color: '#3b82f6', bg: '#eff6ff' },
  { label: 'Python', color: '#6366f1', bg: '#eef2ff' },
  { label: 'PyTorch', color: '#10b981', bg: '#ecfdf5' },
  { label: 'PhysX', color: '#f59e0b', bg: '#fffbeb' },
  { label: 'Nav2', color: '#3b82f6', bg: '#eff6ff' },
  { label: 'LLMs', color: '#6366f1', bg: '#eef2ff' },
];

function TechStrip() {
  return (
    <section className={styles.techStrip}>
      <div className={styles.techInner}>
        <p className={styles.techLabel}>Technologies covered in this textbook</p>
        <div className={styles.techList}>
          {techStack.map((t) => (
            <span
              key={t.label}
              className={styles.techBadge}
              style={{ color: t.color, background: t.bg, border: `1px solid ${t.color}22` }}>
              {t.label}
            </span>
          ))}
        </div>
      </div>
    </section>
  );
}

// ─── Modules ──────────────────────────────────────────────────────────────────

const modules = [
  {
    number: '01',
    emoji: '📡',
    color: '#3b82f6',
    bg: '#eff6ff',
    border: '#bfdbfe',
    title: 'The Robotic Nervous System',
    subtitle: 'ROS 2',
    weeks: 'Weeks 1–3',
    chapters: '6 Chapters',
    description:
      'Master ROS 2 fundamentals — nodes, topics, services, and packages. Integrate LIDAR and IMU sensors, build Python agents, and model robots with URDF.',
    topics: ['LIDAR & IMU Sensors', 'Nodes & Topics', 'Services & Packages', 'URDF Modeling'],
    path: '/hackathon_textbook_ai_robotics/docs/module-1-robotic-nervous-system',
  },
  {
    number: '02',
    emoji: '🌐',
    color: '#6366f1',
    bg: '#eef2ff',
    border: '#c7d2fe',
    title: 'The Digital Twin',
    subtitle: 'Gazebo & Unity',
    weeks: 'Weeks 4–5',
    chapters: '4 Chapters',
    description:
      'Build high-fidelity digital twins using Gazebo physics simulation and Unity HDRP rendering. Simulate cameras, LIDAR, and IMU with physical accuracy.',
    topics: ['Physics Simulation', 'Collision Detection', 'Unity HDRP & PBR', 'Sensor Simulation'],
    path: '/hackathon_textbook_ai_robotics/docs/module-2-digital-twin',
  },
  {
    number: '03',
    emoji: '🧠',
    color: '#10b981',
    bg: '#ecfdf5',
    border: '#a7f3d0',
    title: 'The AI-Robot Brain',
    subtitle: 'NVIDIA Isaac™',
    weeks: 'Weeks 6–8',
    chapters: '6 Chapters',
    description:
      'Leverage NVIDIA Isaac Sim for photorealistic simulation, hardware-accelerated VSLAM with Nav2, and GPU-powered reinforcement learning with domain randomization.',
    topics: ['Isaac Sim Setup', 'VSLAM & Nav2', 'Isaac Gym RL', 'Domain Randomization'],
    path: '/hackathon_textbook_ai_robotics/docs/module-3-ai-robot-brain',
  },
  {
    number: '04',
    emoji: '👁️',
    color: '#f59e0b',
    bg: '#fffbeb',
    border: '#fde68a',
    title: 'Vision-Language-Action',
    subtitle: 'VLA Systems',
    weeks: 'Weeks 9–13',
    chapters: '10 Chapters',
    description:
      'Integrate voice commands via OpenAI Whisper, LLM-based cognitive planning, and deploy an autonomous humanoid robot from simulation to the real world.',
    topics: ['OpenAI Whisper', 'LLM Planning', 'ROS 2 Actions', 'Humanoid Deployment'],
    path: '/hackathon_textbook_ai_robotics/docs/module-4-vision-language-action',
  },
];

function ModuleCard({ mod }: { mod: typeof modules[0] }) {
  return (
    <Link to={mod.path} style={{ textDecoration: 'none', color: 'inherit', display: 'block' }}>
      <div className={styles.moduleCard}>
        <div className={styles.moduleCardTop} style={{ borderTop: `3px solid ${mod.color}` }}>
          <div className={styles.moduleHeaderRow}>
            <span className={styles.moduleEmoji}>{mod.emoji}</span>
            <div>
              <span className={styles.moduleNumber} style={{ color: mod.color }}>
                Module {mod.number}
              </span>
              <span className={styles.moduleWeeks}>{mod.weeks} · {mod.chapters}</span>
            </div>
          </div>
          <h3 className={styles.moduleTitle}>{mod.title}</h3>
          <p className={styles.moduleSubtitle} style={{ color: mod.color }}>{mod.subtitle}</p>
          <p className={styles.moduleDesc}>{mod.description}</p>
        </div>

        <div className={styles.moduleCardBottom}>
          <div className={styles.moduleTopics}>
            {mod.topics.map((t) => (
              <span
                key={t}
                className={styles.moduleTopic}
                style={{ color: mod.color, background: mod.bg, border: `1px solid ${mod.border}` }}>
                {t}
              </span>
            ))}
          </div>
          <span className={styles.moduleLink} style={{ color: mod.color }}>
            Explore Module
            <svg width="14" height="14" viewBox="0 0 16 16" fill="none">
              <path d="M3 8h10M9 4l4 4-4 4" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round" />
            </svg>
          </span>
        </div>
      </div>
    </Link>
  );
}

function ModulesSection() {
  return (
    <section className={styles.modulesSection}>
      <div className={styles.sectionInner}>
        <div className={styles.sectionHeader}>
          <span className={styles.sectionBadge}>Curriculum</span>
          <h2 className={styles.sectionTitle}>4 Modules, 13 Weeks</h2>
          <p className={styles.sectionSubtitle}>
            A structured path from ROS 2 fundamentals to autonomous humanoid deployment
          </p>
        </div>
        <div className={styles.modulesGrid}>
          {modules.map((mod) => (
            <ModuleCard key={mod.number} mod={mod} />
          ))}
        </div>
      </div>
    </section>
  );
}

// ─── Features ─────────────────────────────────────────────────────────────────

const features = [
  { icon: '📚', title: 'Comprehensive Coverage', desc: '26 chapters covering every aspect of Physical AI — from sensors and simulation to autonomous deployment.' },
  { icon: '💻', title: 'Hands-On Code', desc: 'Over 100 real code examples in Python, C++, YAML, and C# with working robotics implementations.' },
  { icon: '🤖', title: 'Industry Tools', desc: 'ROS 2, NVIDIA Isaac, Gazebo, Unity — the exact tools used in top robotics research labs worldwide.' },
  { icon: '🎯', title: 'Clear Objectives', desc: 'Every chapter has defined learning objectives and key takeaways so you always know what you have mastered.' },
  { icon: '🔬', title: 'Research-Grade', desc: 'Content aligned with cutting-edge research in sim-to-real transfer, RL, and Vision-Language-Action systems.' },
  { icon: '🚀', title: 'Capstone Project', desc: 'Deploy a fully autonomous humanoid robot — from simulation in Isaac Sim to real-world testing.' },
];

function FeaturesSection() {
  return (
    <section className={styles.featuresSection}>
      <div className={styles.sectionInner}>
        <div className={styles.sectionHeader}>
          <span className={styles.sectionBadge}>Why This Textbook</span>
          <h2 className={styles.sectionTitle}>Built for Physical AI Engineers</h2>
          <p className={styles.sectionSubtitle}>
            Everything you need to go from zero to autonomous humanoid deployment
          </p>
        </div>
        <div className={styles.featuresGrid}>
          {features.map((f) => (
            <div key={f.title} className={styles.featureCard}>
              <div className={styles.featureIcon}>{f.icon}</div>
              <h3 className={styles.featureTitle}>{f.title}</h3>
              <p className={styles.featureDesc}>{f.desc}</p>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

// ─── CTA ──────────────────────────────────────────────────────────────────────

function CTASection() {
  return (
    <section className={styles.ctaSection}>
      <div className={styles.ctaInner}>
        <h2 className={styles.ctaTitle}>Ready to Build the Future?</h2>
        <p className={styles.ctaSubtitle}>
          Start your journey into Physical AI and Humanoid Robotics today — completely free.
        </p>
        <div className={styles.heroButtons}>
          <Link className={styles.btnPrimary} to="/docs/intro">
            Begin Chapter 1
            <svg width="16" height="16" viewBox="0 0 16 16" fill="none">
              <path d="M3 8h10M9 4l4 4-4 4" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round" />
            </svg>
          </Link>
          <Link className={styles.btnSecondary} to="/docs/quickstart">
            Quick Start Guide
          </Link>
        </div>
      </div>
    </section>
  );
}

// ─── Page Export ──────────────────────────────────────────────────────────────

export default function Home(): ReactNode {
  const { siteConfig } = useDocusaurusContext();
  return (
    <Layout
      title={siteConfig.title}
      description="AI-Native Textbook for Physical AI & Humanoid Robotics covering ROS 2, Gazebo, NVIDIA Isaac, and Vision-Language-Action systems">
      <Hero />
      <TechStrip />
      <ModulesSection />
      <FeaturesSection />
      <CTASection />
    </Layout>
  );
}