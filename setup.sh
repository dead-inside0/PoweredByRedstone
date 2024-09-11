#!/bin/zsh

brew install openjdk openjdk@8 openjdk@11

echo 'export PATH="/usr/local/opt/openjdk/bin:$PATH"' >> ~/.zshrc
echo 'export PATH="/usr/local/opt/openjdk@8/bin:$PATH"' >> ~/.zshrc
echo 'export PATH="/usr/local/opt/openjdk@11/bin:$PATH"' >> ~/.zshrc

brew install temurin

brew install android-commandlinetools
echo 'export ANDROID_HOME="/usr/local/share/android-commandlinetools"'

yes | sdkmanager --licenses
