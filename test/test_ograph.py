import osm_request


class TestOGraph:
    def test_ograph_can_create_dir_if_not_exists(self):
        path_to_test_dir_that_doesnt_already_exist = "/tmp/test_graphml_files"
        test_ograph = osm_request.OGraph(
            "Harlem, NY",
            preview=False,
            path_to_graphml_files=path_to_test_dir_that_doesnt_already_exist,
        )

        assert test_ograph.path_to_graphml_files.exists()
        test_ograph.path_to_graphml_files.rmdir()

    def test_ograph_can_persist_requested_omsnx_graph(self, test_ograph):
        test_ograph.request_graph_from_osmnx(persist=True)

        path_to_saved_osmn_graph_obj = test_ograph.path_to_saved_osmnx_graph_obj

        assert path_to_saved_osmn_graph_obj.exists()
