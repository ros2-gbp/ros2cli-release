%bcond_without tests
%bcond_without weak_deps

%global __os_install_post %(echo '%{__os_install_post}' | sed -e 's!/usr/lib[^[:space:]]*/brp-python-bytecompile[[:space:]].*$!!g')
%global __provides_exclude_from ^/opt/ros/iron/.*$
%global __requires_exclude_from ^/opt/ros/iron/.*$

Name:           ros-iron-ros2param
Version:        0.25.2
Release:        1%{?dist}%{?release_suffix}
Summary:        ROS ros2param package

License:        Apache License 2.0
Source0:        %{name}-%{version}.tar.gz

Requires:       ros-iron-rcl-interfaces
Requires:       ros-iron-rclpy
Requires:       ros-iron-ros2cli
Requires:       ros-iron-ros2node
Requires:       ros-iron-ros2service
Requires:       ros-iron-ros-workspace
BuildRequires:  python%{python3_pkgversion}-devel
BuildRequires:  ros-iron-ros-workspace
Provides:       %{name}-devel = %{version}-%{release}
Provides:       %{name}-doc = %{version}-%{release}
Provides:       %{name}-runtime = %{version}-%{release}

%if 0%{?with_tests}
BuildRequires:  python%{python3_pkgversion}-pytest
BuildRequires:  python%{python3_pkgversion}-pytest-timeout
BuildRequires:  ros-iron-ament-copyright
BuildRequires:  ros-iron-ament-flake8
BuildRequires:  ros-iron-ament-pep257
BuildRequires:  ros-iron-ament-xmllint
BuildRequires:  ros-iron-launch
BuildRequires:  ros-iron-launch-ros
BuildRequires:  ros-iron-launch-testing
BuildRequires:  ros-iron-launch-testing-ros
%endif

%description
The param command for ROS 2 command line tools.

%prep
%autosetup -p1

%build
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/iron/setup.sh" ]; then . "/opt/ros/iron/setup.sh"; fi
%py3_build

%install
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/iron/setup.sh" ]; then . "/opt/ros/iron/setup.sh"; fi
%py3_install -- --prefix "/opt/ros/iron"

%if 0%{?with_tests}
%check
# Look for a directory with a name indicating that it contains tests
TEST_TARGET=$(ls -d * | grep -m1 "\(test\|tests\)" ||:)
if [ -n "$TEST_TARGET" ] && %__python3 -m pytest --version; then
# In case we're installing to a non-standard location, look for a setup.sh
# in the install tree and source it.  It will set things like
# CMAKE_PREFIX_PATH, PKG_CONFIG_PATH, and PYTHONPATH.
if [ -f "/opt/ros/iron/setup.sh" ]; then . "/opt/ros/iron/setup.sh"; fi
%__python3 -m pytest $TEST_TARGET || echo "RPM TESTS FAILED"
else echo "RPM TESTS SKIPPED"; fi
%endif

%files
/opt/ros/iron

%changelog
* Fri Jul 14 2023 Audrow Nash <audrow@openrobotics.org> - 0.25.2-1
- Autogenerated by Bloom

* Thu May 11 2023 Audrow Nash <audrow@openrobotics.org> - 0.25.1-1
- Autogenerated by Bloom

* Thu Apr 20 2023 Audrow Nash <audrow@openrobotics.org> - 0.25.0-2
- Autogenerated by Bloom

* Tue Apr 18 2023 Audrow Nash <audrow@openrobotics.org> - 0.25.0-1
- Autogenerated by Bloom

* Wed Apr 12 2023 Audrow Nash <audrow@openrobotics.org> - 0.24.1-1
- Autogenerated by Bloom

* Tue Apr 11 2023 Audrow Nash <audrow@openrobotics.org> - 0.24.0-1
- Autogenerated by Bloom

* Tue Mar 21 2023 Audrow Nash <audrow@openrobotics.org> - 0.23.0-2
- Autogenerated by Bloom

