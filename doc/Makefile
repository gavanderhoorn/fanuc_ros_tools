.PHONY: html apidoc clean

html: apidoc
	make -C _build html

apidoc:
	sphinx-apidoc -F -o _build ../fanuc_ros_tools
	sed -i "s/_build/./g" _build/Makefile

clean:
	-rm -rf _build
