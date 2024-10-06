## Installing large datasets with GIT LFS

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

2. Track the large files
    ```bash
    git lfs track "*.zip"
    ```
    Add the extension of the files that you want to track using LFS. The command should create a .gitattributes file in the directory.

3. Push LFS
    ```bash
    git lfs push --all origin main
    ```
    