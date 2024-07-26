#!/bin/bash

# Get the directory of the current script
BASE_DIR="$(pwd)"

# Function to clone a repository and switch to a specific branch if needed
clone_repo() {
  local repo_url=$1
  local branch=$2
  local repo_name=$(basename -s .git "$repo_url")

  git clone "$repo_url"
  if [ -n "$branch" ]; then
    cd "$repo_name" || exit
    git switch "$branch"
    cd ..
  fi
}

# Navigate to the base directory
cd "$BASE_DIR" && cd ..

# Clone the repositories
clone_repo "https://github.com/CNR-STIIMA-IRAS/cnr_logger.git" "modern_cmake"
clone_repo "https://github.com/CNR-STIIMA-IRAS/cnr_yaml.git"
clone_repo "https://github.com/CNR-STIIMA-IRAS/cnr_param.git" "no-ament-only-cmake"
clone_repo "https://github.com/JRL-CARI-CNR-UNIBS/cnr_class_loader.git"