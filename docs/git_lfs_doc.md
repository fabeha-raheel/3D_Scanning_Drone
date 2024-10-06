## Installing large datasets on GitHub with GIT LFS

1. Install Git LFS
    ```bash
    git lfs install
    ```
    If above command gives the following error: ```git: 'lfs' is not a git command. See 'git --help'.```,
    Run the following commands (for Ubuntu):
    ```bash
    curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash
    sudo apt-get install git-lfs
    ```
    And then run the above command.
    Solution available at: https://stackoverflow.com/questions/48734119/git-lfs-is-not-a-git-command-unclear

    This step needs to be executed only once on any machine.

2. Track the large files
    ```bash
    git lfs track "datasets/zip_files/*.zip"
    ```
    Add the extension of the files that you want to track using LFS. The command should create a .gitattributes file in the directory.

3. Add and Commit the .gitattributes file
    ```bash
    git add .gitattributes
    git commit -m "Track dataset zip files using git-lfs"
    ```

4. Add and commit the large files and push to remote repo:
    ```bash
    git add datasets/zip_files/*.zip
    git commit -m "Add large datasets"
    git push origin main
    ```
    