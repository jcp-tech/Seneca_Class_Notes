cd ..
find . -name .git -type d -exec rm -fr {} \;
rm .git-credentials
rm .gitconfig
