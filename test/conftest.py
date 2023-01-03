import pytest
import osm_request


@pytest.fixture(scope="session")
def test_ograph(tmp_path_factory):
    """
    This fixture creates a test OGraph object that can be used to test the OGraph class.
    It depends on the temporary path factory fixture provided by pytest to generate
    a unique temporary directory with the specified name.
    """
    test_graph = osm_request.OGraph(
        "Harlem, NY",
        preview=False,
        path_to_graphml_files=tmp_path_factory.mktemp("test_graphml_files").as_posix(),
    )
    test_graph.request_graph_from_osmnx()
    return test_graph
