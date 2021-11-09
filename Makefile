# Self-documenting Makefile by prwhite
# https://gist.github.com/prwhite/8168133
# https://gist.github.com/prwhite/8168133#gistcomment-1716694
# https://gist.github.com/prwhite/8168133#gistcomment-1737630

help: ## Show this help message.
	@echo "Usage: make [target] ..."
	@echo
	@echo "Targets:"
	@grep --color=auto -F "## " $(MAKEFILE_LIST) | grep --color=auto -F -v grep | sed -e "s/\\$$//" | sed -e "s/##//" | column -c2 -t -s :
	@grep "##@[^ \"]*" $(MAKEFILE_LIST) | grep --color=auto -F -v grep | sed -e "s/^.*##@\\([a-z][a-z]*\\).*\$$/\1/" | sed "/^\\$$/d" | sort | uniq | xargs -n 1 -I'{}' bash -c "echo; echo {} targets:; grep '##@{}' Makefile | sed -e 's/##@{}//' | column -c2 -t -s :"


build: ##@Build Build a Python3 wheel.
	python3 setup.py build bdist_wheel --python-tag py3

develop: ##@Developer Install the package as link to this repository.
	python3 setup.py develop --user

install: ##@Install Install the package for current user.
	python3 setup.py install --user

uninstall: ##@Install Uninstall the package.
	python3 -m pip uninstall ng_trajectory

reinstall: ##@Install Reinstall the package
reinstall: uninstall install
