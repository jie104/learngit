# Git Hooks说明

## 本地安装方法
1. 进入`git-hooks/`目录
2. 执行`./install_git_hooks.sh`即可

### pre-commmit

在`git commit`执行之前，会先执行`.git/hooks/pre-commit`脚本，如果该脚本返回值非0，那么`git commit`操作就会取消。

在`pre-commit`脚本中，我们添加了执行`cpplint.py`的相关代码，这样每次提交前，`cpplint.py`会检查本次修改的源代码文件。
如果不满足`cpplint.py`的静态检查条件，那么将无法提交。

临时跳过hooks执行的方法： 执行`git commit`命令时添加`-n`参数

### commit-msg

1. 在commit msg最开始中添加当前分支名（若当前分支为master、develop则不添加）
2. 在commit msg最后添加`Code Static Check Passed`文本标记


## 服务器端安装方法（Gitlab）

### Create a custom Git hook for a repository
Server-side Git hooks are typically placed in the repository’s hooks subdirectory. In GitLab, hook directories
are are symlinked to the gitlab-shell hooks directory for ease of maintenance between gitlab-shell upgrades.
Custom hooks are implemented differently, but the behavior is exactly the same once the hook is created.
Follow the steps below to set up a custom hook for a repository:

1. Pick a project that needs a custom Git hook.
2. On the GitLab server, navigate to the project’s repository directory. For an installation from source 
the path is usually `/home/git/repositories/<group>/<project>.git`. For Omnibus installs the path is usually
`/var/opt/gitlab/git-data/repositories/<group>/<project>.git`.
3. Create a new directory in this location called `custom_hooks`.
4. Inside the new custom_hooks directory, create a file with a name matching the hook type. For a pre-receive
hook the file name should be pre-receive with no extension.
5. **Make the hook file executable and make sure it’s owned by git**.
6. Write the code to make the Git hook function as expected. Hooks can be in any language. Ensure the ‘shebang’
at the top properly reflects the language type. For example, if the script is in Ruby the shebang will probably
be `#!/usr/bin/env ruby`.

That’s it! Assuming the hook code is properly implemented the hook will fire as appropriate.

### Set a global Git hook for all repositories
To create a Git hook that applies to all of your repositories in your instance, set a global Git hook. 
Since all the repositories’ hooks directories are symlinked to gitlab-shell’s hooks directory, 
adding any hook to the gitlab-shell hooks directory will also apply it to all repositories. 
Follow the steps below to properly set up a custom hook for all repositories:

1. On the GitLab server, navigate to the configured custom hook directory. The default is in the 
gitlab-shell directory. The gitlab-shell hook directory for an installation from source the path is usually
`/home/git/gitlab-shell/hooks`. For Omnibus installs the path is usually 
`/opt/gitlab/embedded/service/gitlab-shell/hooks`. To look in a different directory for the global custom hooks,
set custom_hooks_dir in the gitlab-shell config. For Omnibus installations, this can be set in gitlab.rb;
and in source installations, this can be set in `gitlab-shell/config.yml`.
2. Create a new directory in this location. Depending on your hook, it will be either a pre-receive.d,
post-receive.d, or update.d directory.

3. Inside this new directory, add your hook. Hooks can be in any language. Ensure the ‘shebang’ at the top 
properly reflects the language type. For example, if the script is in Ruby the shebang will probably be
`#!/usr/bin/env ruby`.
4. Make the hook file executable and make sure it’s owned by Git.

Now test the hook to see that it’s functioning properly.


## NOTE
- 当文件为”UTF-8 BOM“格式时，第一行需要空行，不然cpplint检测会报错。