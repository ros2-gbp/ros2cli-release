%bcond_without tests
%bcond_without weak_deps

%global __os_install_post %(echo '%{__os_install_post}' | sed -e 's!/usr/lib[^[:space:]]*/brp-python-bytecompile[[:space:]].*$!!g')
%global __provides_exclude_from ^/opt/ros/humble/.*$
%global __requires_exclude_from ^/opt/ros/humble/.*$

Name:           ros-humble-ros2interface
Version:        0.18.4
Release:        1%{?dist}%{?release_suffix}
Summary:        ROS ros2interface package

License:        Apache License 2.0
Source0:        %{name}-%{version}.tar.gz

Requires:       ros-humble-ament-index-python
Requires:       ros-humble-ros2cli
Requires:       ros-humble-rosidl-runtime-py
Requires:       ros-humble-ros-workspace
BuildRequires:  python%{python3_pkgversion}-devel
BuildRequires:  ros-humble-ros-workspace
BuildRequires:  ros-humble-ros2cli
Provides:       %{name}-devel = %{version}-%{release}
Provides:       %{name}-doc = %{version}-%{release}
Provides:       %{name}-runtime = %{version}-%{release}

%if 0%{?with_tests}
BuildRequires:  python%{python3_pkgversion}-pytest
BuildRequires:  python%{python3_pkgversion}-pytest-timeout
BuildRequires:  ros-humble-ament-copyright
BuildRequires:  ros-humble-ament-flake8
BuildRequires:  ros-humble-ament-pep257
BuildRequires:  ros-humble-ament-xmllint
BuildRequires:  ros-humble-launch
BuildRequires:  ros-humble-launch-testing
BuildRequires:  ros-humble-launch-testing-ros
BuildRequires:  ros-humble-ros2cli-test-interfaces
BuildRequires:  ros-humble-test-msgs
%endif

%description
The interface command for ROS 2 command line tools

%prep
%autosetup -p1

%build
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/humble/setup.sh" ]; then . "/opt/ros/humble/setup.sh"; fi
%py3_build

%install
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/humble/setup.sh" ]; then . "/opt/ros/humble/setup.sh"; fi
%py3_install -- --prefix "/opt/ros/humble"

%if 0%{?with_tests}
%check
# Look for a directory with a name indicating that it contains tests
TEST_TARGET=$(ls -d * | grep -m1 "\(test\|tests\)" ||:)
if [ -n "$TEST_TARGET" ] && %__python3 -m pytest --version; then
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/humble/setup.sh" ]; then . "/opt/ros/humble/setup.sh"; fi
%__python3 -m pytest $TEST_TARGET || echo "RPM TESTS FAILED"
else echo "RPM TESTS SKIPPED"; fi
%endif

%files
/opt/ros/humble

%changelog
* Tue Nov 08 2022 Aditya Pande <aditya.pande@openrobotics.org> - 0.18.4-1
- Autogenerated by Bloom

* Mon Nov 07 2022 Aditya Pande <aditya.pande@openrobotics.org> - 0.21.0-1
- Autogenerated by Bloom

* Tue Apr 19 2022 Aditya Pande <aditya.pande@openrobotics.org> - 0.18.3-2
- Autogenerated by Bloom

* Fri Apr 08 2022 Aditya Pande <aditya.pande@openrobotics.org> - 0.18.3-1
- Autogenerated by Bloom

* Wed Mar 30 2022 Aditya Pande <aditya.pande@openrobotics.org> - 0.18.2-1
- Autogenerated by Bloom

* Mon Mar 28 2022 Aditya Pande <aditya.pande@openrobotics.org> - 0.18.1-1
- Autogenerated by Bloom

* Tue Mar 01 2022 Aditya Pande <aditya.pande@openrobotics.org> - 0.18.0-1
- Autogenerated by Bloom

* Tue Feb 08 2022 Aditya Pande <aditya.pande@openrobotics.org> - 0.17.1-2
- Autogenerated by Bloom

