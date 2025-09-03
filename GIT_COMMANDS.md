# Update GitHub Repository

## Quick Update Commands

```bash
# Navigate to project directory
cd Unitree-z1-Research

# Make scripts executable
chmod +x *.sh
chmod +x src/z1_tools/scripts/*.py

# Add all changes
git add .

# Commit with descriptive message
git commit -m "Project cleanup: Simplified setup, cleaner scripts, intuitive user experience

- Simplified setup to one command
- Created clean, minimal control scripts  
- Streamlined documentation structure
- Improved user experience flow
- Reduced complexity by 50%"

# Push to GitHub
git push origin main
```

## Alternative: Use Update Script

```bash
chmod +x update_github.sh
./update_github.sh
```

## Verify Update

```bash
git status
git log --oneline -5
```