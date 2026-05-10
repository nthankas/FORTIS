# Copyright 2017 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from pathlib import Path

from ament_flake8.main import main_with_errors
import pytest


# Lint only the source tree this scaffold belongs to. Resolved from
# __file__ so the lint target is independent of the cwd pytest was
# launched from -- without this, running pytest from the workspace root
# would scan every .py file in the repo (sim/, build/, install/, ...).
PKG_DIR = Path(__file__).resolve().parent.parent


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    rc, errors = main_with_errors(argv=[str(PKG_DIR)])
    assert rc == 0, \
        'Found %d code style errors / warnings:\n' % len(errors) + \
        '\n'.join(errors)
