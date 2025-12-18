# Quickstart Guide: Setting Up ROS 2 Education Docusaurus Site

## Prerequisites
- Node.js 18+ installed
- npm or yarn package manager
- Git for version control

## Installation Steps

### 1. Initialize Docusaurus Project
```bash
npm init docusaurus@latest website classic
```

### 2. Navigate to Website Directory
```bash
cd website
```

### 3. Install Additional Dependencies (if needed)
```bash
npm install
```

### 4. Create Module 1 Directory Structure
```bash
mkdir -p docs/module-1
```

### 5. Create the Three Chapters
Create the following files in the `docs/module-1/` directory:
- `intro-to-ros2.md`
- `ros2-communication.md`
- `urdf-robot-structure.md`

### 6. Configure Sidebar Navigation
Update `sidebars.js` to include the new module and chapters.

### 7. Start Local Development Server
```bash
npm start
```

The site will be available at http://localhost:3000

## Key Configuration Files
- `docusaurus.config.js` - Main site configuration
- `sidebars.js` - Navigation structure
- `docs/` - All documentation content
- `static/` - Static assets like images

## Next Steps
1. Add your content to the chapter files
2. Customize the site theme and styling
3. Configure deployment settings for GitHub Pages
4. Test the site locally before deploying