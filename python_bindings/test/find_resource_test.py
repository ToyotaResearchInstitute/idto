import os

import pytest
from pyidto import FindIdtoResource


def test_find_resource():
    """Test the FindResource function."""
    with pytest.raises(RuntimeError):
        FindIdtoResource("non_existent_file")

    spinner_urdf = FindIdtoResource("idto/models/hopper.urdf")
    assert os.path.isfile(spinner_urdf)

    nonexistent = FindIdtoResource("idto/non_existent_file")
    assert not os.path.isfile(nonexistent)
