# Template Lib

## Usage
Build project
```shell
jarvis_make
```

Build deb
```shell
jarvis_make deb
```

deb packages:
* template_[version]_amd64.deb
    * library files
* template-devel_[version]_amd64.deb
    * header files and pkgconfig file
    * sample code
* template-dbgsym_[version]_amd64.deb
    * library's debug files

## Make your own library package
* include dir
    * Save header files here, which you want export to others
* template.pc.in
    1. Change file name to `{your_project_name}.pc.in`
    1. Change sections in your pkgconfig files
        * Name - `{your_project_name}`
        * Requires - Your depend libraris which provide pkgconfig file
            * Same with contents of `pkg_check_modules` in CMakeLists.txt
        * Libs - `-l{your_project_name}`
* CMakeLists.txt
    1. Change project name
        * `project({your_project_name} VERSION 0.1.0)`
        *
            ```
            configure_file({your_project_name}.pc.in {your_project_name}.pc)
            install(FILES ${CMAKE_CURRENT_BINARY_DIR}/{your_project_name}.pc
                COMPONENT devel
                DESTINATION ${CMAKE_INSTALL_PREFIX}/share/pkgconfig
            )
            ```
    1. Set package owner name
        * `set(CPACK_PACKAGE_CONTACT "{Your Name}@sg.cambricon.com")`
    1. Set package dependency
        * `set(CPACK_DEBIAN_PACKAGE_DEPENDS "{Depend pkg 1}, {Depend pkg 2}")`
* sample
    * Change project name in CMakeLists.txt, package.xml, and .launch file
    * Write sample code use your library, and get data from ros messages
    * Verification:
        1. Install devel package in your dev. enviroment
        1. Build sample code in install directory [/usr/share/jarvis/[project_name]/sample]
        1. Running sample binary