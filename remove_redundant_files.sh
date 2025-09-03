#!/bin/bash

# Remove redundant documentation files
echo "🧹 Cleaning up redundant files..."

# Remove outdated/redundant docs
rm -f README_SCRIPTS.md
rm -f SETUP_UBUNTU_20.04.md  
rm -f CLEAN_PROJECT_SUMMARY.md
rm -f UBUNTU_COMPATIBILITY_CHECK.md
rm -f GIT_COMMANDS.md
rm -f COMPLETE_FIX.md
rm -f fix_gazebo_issues.sh
rm -f update_github.sh
rm -f upload_to_github.sh
rm -f push_to_main.sh
rm -f error.md

echo "✅ Redundant files removed"
echo "📋 Remaining core files:"
echo "  - README.md (main info)"
echo "  - START_HERE.md (quick start)"  
echo "  - TWO_TERMINAL_USAGE.md (complete reference)"
echo "  - SIMULATION_VS_REAL.md (safety info)"
echo "  - setup_and_run.sh (setup)"
echo "  - quick_start.sh (daily use)"