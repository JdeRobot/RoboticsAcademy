# Jekyll Local Installation

## Prerequisites

### Installing Ruby on Ubuntu

First of all, we need to install all the dependencies typing:

```bash
sudo apt-get install ruby-full build-essential zlib1g-dev
```

After that, we need to set up a gem installation directory for your user account. The following commands will add environment variables to your `~/.bashrc` file to configure the gem installation path. Run them now:

```bash
echo '# Install Ruby Gems to ~/gems' >> ~/.bashrc
echo 'export GEM_HOME="$HOME/gems"' >> ~/.bashrc
echo 'export PATH="$HOME/gems/bin:$PATH"' >> ~/.bashrc
source ~/.bashrc
```

Finally, we install Jekyll:

```bash
gem install jekyll bundler
```

Notice that we don't use the `root` user :-)

## Running Jekyll Serve

By default, the Jekyll server is launched with the following command (which is the one indicated on your website).

```bash
bundle exec jekyll serve
```

If in the process of building the server there is a dependency problem, for example, there is a missing library to install, it is **necessary to delete** the `Gemfile.lock` file so that it is rebuilt with the installed dependency. This list of dependencies is found in the `Gemfile` file (in Python it would be equivalent to the `requirements.txt` file) and the version of each of the installed gems (packages) is specified. Having a list of dependencies is important for future updates as well as knowing the libraries needed to run the server. Once the `Gemfile.lock` file is deleted, the command shown above is launched again and the dependency errors should end.

## Notes for exercise cards.

- Teaser Images size: multiple of 600x400px


## FAQ

- Error building Jekyll server: 

```bash
jekyll build --incremental --verbose
```