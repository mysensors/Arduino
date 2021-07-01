# Development Tools

### Overview

This directory hosts MySensors development tools. The following
conventions are employed to facilitate consistent re-use/invocation
across modalities (e.g. local development, continuous integration,
editor linters, etc.)

1.  All common tools are hosted and managed in
    the tools directory (used for both local development
    and continuous integration)
2.  Each tool comprises a directory, akin to a bundle,
    that encapsulates declarative command-line options,
    configuration files and a run script
3.  A single bootstrap script configures a
    development environment
4.  A lightweight runtime provides a common set of
    convenience functions and variables to all scripts
5.  Support for all MySensors development environments

### Usage

The [boostrap-dev](bootstrap-dev.sh) script completely configures a
development environment. The boostrap-dev script validates development
prerequisites such as verifying the git repo ```origin``` &
```upstream``` remotes, tools needed for the git client hooks,
installing local commit hooks, and creating git aliases, etc.

```shell-script
$ cd MySensors    # git repo
$ .mystools/bootstrap-dev.sh
Checking operating system support: darwin16...
Checking github 'origin' & 'upstream' remotes...
Checking tool/utility prerequisites...
Installing git client-side hooks...
Configuring git aliases for running mysensor tool bundles...
Successfully configured your repo for MySensors development... Thanks for your support!
$
```
**Note:**  If the bootstrap can not find required command-line
utilities, you will be requested to install the required tools and
re-run bootstrap.sh.  See [Installation instructions for prerequisite
tools](#installtools).

Once the bootstrapping process is complete, a git alias for each tool
is available to conveniently run the tool with the pre-configured
mysensors options and settings (no need to remember all those
command-line options or where the configuration files reside).
Furthermore, you can run the command against staged files, unstaged
modified files or a list of files common to most git commands.

```shell-script
$ # Run cppcheck on an individual file
$ git cppcheck core/MyMessage.cpp
$ 
$ # Run cppcheck on all changed files
$ git cppcheck 
$
$ # Run astyle on staged files
$ git astyle --cached
```

Available tool aliases:

* git cppcheck [--cached] [files]
* git astyle [--cached] [files]

Finally, to maintain code quality and a legible git revision history,
two git hooks are installed that will trigger whenever you commit
changes to your local repo.  The hooks will examine your changes to
ensure they satisfy the [MySensors Code Contribution
Guidelines](https://www.mysensors.org/download/contributing) before
you push your changes to GitHub for merging by the core MySensors
team.  Gitler will also enforce the coding guidelines so the hooks are
provided to reduce development cycle times by detecting standards
variances locally before pushing to GitHub.

### <a name="installtools"></a>Installation instructions for prerequisite tools

This first time you run the bootstrap script, it may inform you that
certain development tools are missing from your path or system:

```
Checking operating system support: darwin16...
Checking github 'origin' & 'upstream' remotes...
Checking tool/utility prerequisites...
astyle not installed or not in current path.
cppcheck not installed or not in current path.
One or more required tools not found.  Install required tools and re-run .mystools/bootstrap-dev.sh
```

To finish the bootstrap process, you will need to install the required
tools for your specific operating system as follows.  Currently we use
Astyle 3.1 or later and Cppcheck 2.1 or later.  Once you have
installed AStyle and Cppcheck, re-run bootstrap-dev.sh to finish
configuring your development environment.

##### macOS

```brew install astyle cppcheck```

#### Linux Xenial or later

```apt-get install astyle cppcheck```

#### Linux Trusty or earlier

Note: The apt versions are too old on Trusty so follow the [Linux - Build and Install from Source](#buildFromSource) instructions

##### Windows - WSL
Tested with Ubuntu 18.04 LTS and 19.04
```
$ apt-get install astyle
$ git clone https://github.com/danmar/cppcheck.git
$ cd cppcheck/
$ git checkout 2.1 # or later version if available
$ mkdir build && cd build && cmake .. && cmake --build .
$ sudo make MATCHCOMPILER=yes FILESDIR=/usr/share/cppcheck HAVE_RULES=yes CXXFLAGS="-O2 -DNDEBUG -Wall -Wno-sign-compare -Wno-unused-function" install
```

##### Windows - GitHub Git Shell

###### *IMPORTANT:  Be sure to launch PowerShell As Administrator*

### Install AStyle

```
# Download
iwr 'https://netix.dl.sourceforge.net/project/astyle/astyle/astyle%203.1/AStyle_3.1_windows.zip' -UserAgent [Microsoft.PowerShell.Commands.PSUserAgent]::FireFox -OutFile astyle.3.1.zip

# Unzip the filed & move the C:\Program Files
expand-archive astyle.3.1.zip
mv .\astyle.3.1.zip\AStyle 'C:\Program Files\AStyle'

# Add AStyle to your path
[Environment]::SetEnvironmentVariable("Path", $env:Path + ";C:\Program Files\AStyle\bin")
[Environment]::SetEnvironmentVariable("Path", $env:Path + ";C:\Program Files\AStyle\bin", [EnvironmentVariableTarget]::Machine)
```

### Install Cppcheck x64

```
# 64-bit
iwr 'https://github.com/danmar/cppcheck/releases/download/2.1/cppcheck-2.1-x64-Setup.msi' -UserAgent [Microsoft.PowerShell.Commands.PSUserAgent]::FireFox -OutFile cppcheck-2.1-x64-Setup.msi

# Launch installer to install Cppcheck
& .\cppcheck-2.1-x64-Setup.msi

# Add Cppcheck to your path
[Environment]::SetEnvironmentVariable("Path", $env:Path + ";C:\Program Files\Cppcheck", [EnvironmentVariableTarget]::Machine)
```

### At this point you need to reboot for the path changes to take effect

##### Windows - Cygwin

Run either `Cygwin Setup-x86-64.exe` or `Setup-x86.exe` depending upon your OS. Select and install `astyle` and `cppcheck`.

##### <a name="buildFromSource"></a>Linux - Build and Install from Source

### Install AStyle

```
$ # Download
$ curl -L 'https://sourceforge.net/projects/astyle/files/astyle/astyle%202.05.1/astyle_2.05.1_linux.tar.gz/download' | tar xvz

$ # Compile and install
$ cd astyle/build/gcc && sudo make shared release shared static install
```

### Install Cppcheck
```
$ git clone https://github.com/danmar/cppcheck.git
$ cd cppcheck/
$ git checkout 2.1 # or later version if available)
$ mkdir build && cd build && cmake .. && cmake --build .
$ sudo make MATCHCOMPILER=yes FILESDIR=/usr/share/cppcheck HAVE_RULES=yes CXXFLAGS="-O2 -DNDEBUG -Wall -Wno-sign-compare -Wno-unused-function" install
```

### Documentation

To generate the user documentation an optional set of tools is necessary.

* doxygen
* plantuml
* java (for plantuml)

Call the doxygen helper from the project-root folder to create the documentation.
```
$ ./Documentation/doxygen.sh
```

### Implementation Details

*This section is intended for developers curious about the
implementation details or interested in extending the development
environment to support additional tools, aliases, etc.*

A lightweight [runtime](.bundle_runtime.sh) (less than 70 lines of
code) provides a simple API to consistently run the MySensors toolset
across modalities including, but not limited to, local development and
continuous integration.  The core concept is that each tool
encapsulates all runtime configuration settings in a simple,
stand-alone bundle, that can be executed by the bundle runtime.  The
runtime provides an API to execute a tool bundle along with
convenience functions and environment variables common to all tools.

Each tool bundle is laid out as follows:

```
${TOOLSDIR}                 [e.g. ${GITREPO}/.mystools}]
.../tool                    [e.g. cppcheck, astyle]
....../run.sh               [tool run script]
....../options.sh           [command-line options]
....../config               [supporting config files]
........./some.cfg
```

All bundle runtime dependencies are defined withing the namespace /
context of a git repo so users are free to rename or move repos or
even use multiple copies of the same repo without encountering
intra-repo tool settings/configuration conflicts.

During the bootstrap process, certain keys and aliases are defined
that the runtime leverages.

##### git config keys

* mysensors.toolsdir = .mystools (defined as a repo-relative path - location agnostic)
* mysensors.bootstrap-cksum = \<git sha of bootstrap-dev.sh used to detect an outdated environment>

##### git aliases

* mystoolspath = *returns the absolute path to the tools dir*
* \<bundle name> = *(e.g. cppcheck) runs the tool bundle*

	NOTE: The \<bundle> aliases are auto-generated by enumerating the bundles located
in *mystoolspath*.

While the ```git <tool> args``` API is designed to cover many the
common use cases, tool and hook authors may need to source the
[runtime](.bundle_runtime.sh) into their script context in order to
use the convenience functions and environment variables as follows:

```shell-script
. "$(git config mystoolspath).bundle_runtime.sh"
```

The runtime shim will instantiate the following environment variables
and convenience functions in the script context for use within a tool
or script.

```
TOOLSDIR - absolute path to repo tools dir.  Maintained in
           a location-agnostic fashion (e.g. if the user
           moves the repo, changes tools directory within
           repo, etc.
GITREPO  - absolute path to the git repo
GITDIR   - absolute path to the repo .git directory

runBundle()     - Run a given tool / bundle
is_installed()  - true if a given utility is installed
supported_os()  - true if running on a supported os
log()           - log a message in default color to console
warn()          - log a message in yellow to console
err()           - log a message in red and exit with non-zero status
```

Many of the aforementioned details can be safely ignored if you want
to add a new development tool to the toolset.  Simply create a bundle
that follows the layout above, declare the tool command-line options
in the [bundle]/options.sh and any configuration files in
[bundle]/config/.  While it should not be necessary to alter the
bundle execution behavior beyond declaring command-line options and
configuration files, you can override it by modifing the [bundle]
run.sh script.
