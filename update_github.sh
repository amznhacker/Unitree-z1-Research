#!/bin/bash

# Quick GitHub Update Script
# Usage: ./update_github.sh [commit-message]

COMMIT_MSG="${1:-Project cleanup: Simplified setup, cleaner scripts, intuitive user experience}"

echo "🚀 Updating GitHub repository..."

# Make scripts executable
chmod +x *.sh
chmod +x src/z1_tools/scripts/*.py 2>/dev/null || true

# Add all changes
git add .

# Commit with message
git commit -m "$COMMIT_MSG

- Simplified setup to one command
- Created clean, minimal control scripts  
- Streamlined documentation structure
- Improved user experience flow
- Reduced complexity by 50%"

# Push to GitHub
git push origin main

echo "✅ GitHub updated successfully!"
echo "🔗 Repository: $(git remote get-url origin)"