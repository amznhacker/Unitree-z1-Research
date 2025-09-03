#!/bin/bash

# GitHub Upload Script for Unitree Z1 Workspace
# Usage: ./upload_to_github.sh [repository-name] [your-github-username]

REPO_NAME="${1:-unitree_z1_workspace}"
GITHUB_USERNAME="${2}"

if [ -z "$GITHUB_USERNAME" ]; then
    echo "Usage: $0 [repository-name] [your-github-username]"
    echo "Example: $0 unitree_z1_workspace myusername"
    exit 1
fi

echo "🚀 Uploading Unitree Z1 Workspace to GitHub..."
echo "Repository: $REPO_NAME"
echo "Username: $GITHUB_USERNAME"

# Initialize git if not already done
if [ ! -d ".git" ]; then
    git init
    echo "✅ Git repository initialized"
fi

# Make scripts executable
chmod +x *.sh
chmod +x src/z1_tools/scripts/*.py 2>/dev/null || true
chmod +x src/z1_tools/scripts/*.sh 2>/dev/null || true

# Add all files
git add .
echo "✅ Files staged"

# Commit
git commit -m "Initial commit: Complete Unitree Z1 ROS workspace with automated setup scripts

Features:
- One-command setup and daily startup scripts
- Multiple control methods (keyboard, Xbox, demos)
- 15+ entertainment and utility demos
- Complete ROS workspace with Z1 SDK
- Ubuntu 20.04.6 LTS support
- Comprehensive documentation"

echo "✅ Files committed"

# Set main branch
git branch -M main

# Add remote origin
git remote remove origin 2>/dev/null || true
git remote add origin "https://github.com/$GITHUB_USERNAME/$REPO_NAME.git"
echo "✅ Remote origin set"

echo ""
echo "📋 Next steps:"
echo "1. Create repository on GitHub: https://github.com/new"
echo "   - Repository name: $REPO_NAME"
echo "   - Description: Complete Unitree Z1 robotic arm ROS workspace with automated setup"
echo "   - Make it public"
echo "   - Don't initialize with README (we have one)"
echo ""
echo "2. Then run:"
echo "   git push -u origin main"
echo ""
echo "3. Or run this command to push now:"
echo "   git push -u origin main"
echo ""
echo "🎯 Repository will be available at:"
echo "   https://github.com/$GITHUB_USERNAME/$REPO_NAME"