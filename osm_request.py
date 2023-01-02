import osmnx as ox  # type: ignore
import logging
import pathlib
import typing

DEFAULT_PATH_TO_LOCAL_GRAPHML_FILE_DIR = "graphml_files"


class OGraph:
    """
    A class to store the information of an open graph, and present operations that are available on open graphs.
    """

    def __init__(
        self,
        query,
        preview: bool = False,
        path_to_graphml_files: str = DEFAULT_PATH_TO_LOCAL_GRAPHML_FILE_DIR,
    ):
        """
        Silently sends a geo-codable query to OSMNX for the graph of roads, persists the result,
        and loads .graphml files locally if they exist in the graph_files/ folder

        :param query: str: geo-codable query such as "Harlem, NY" or "Kigali, Rwanda"
        :param preview: bool: will show graph at run-time if True else False
        :param persist: bool: whether to persist the retrieved OSMNX graph locally
        """
        self.query = query
        self.preview = preview
        self.path_to_graphml_files = pathlib.Path(
            path_to_graphml_files
        ) or pathlib.Path(DEFAULT_PATH_TO_LOCAL_GRAPHML_FILE_DIR)
        self.graph_name = (
            self.query.lower().replace(",", "").replace(" ", "_") + ".graphml"
        )
        self.local_files: typing.List[str] = []
        self.dir_check()
        self.init_graph = None
        # self.init_graph = self.request()
        # self.fig, self.axis = None, None
        # self.ax, self.G = self.project_axis()

    def request_graph_from_osmnx(self, persist=False) -> None:
        """
        Negotiates a query to OSMNX if no local stored file else loads local file

        :return: result:
        """
        self.local_files = list(self.path_to_graphml_files.iterdir())  # type: ignore
        if self.graph_name in self.local_files:
            logging.info(f"Loading {self.graph_name} from local storage...")
            self.init_graph = ox.load_graphml(
                f"{self.path_to_graphml_files}/{self.graph_name}"
            )
        else:
            logging.info(f"Requesting {self.graph_name} from OSMNX...")
            self.init_graph = self.send_query(persist=persist)
        return

    def dir_check(self) -> None:
        """
        This method checks if the targeted location to store the GraphML files exists.
        If the directory doesn't exist, it is created.

        :return:
        """
        if not self.path_to_graphml_files.exists():
            logging.info(
                f"Could not find directory {self.path_to_graphml_files.as_posix()}. Creating directory {self.path_to_graphml_files.as_posix()}..."
            )
            self.path_to_graphml_files.mkdir()
        return

    def send_query(self, persist=False):
        """
        Request a graph from OSMNX

        :return: G: OSMN Graph object
        """
        # query graph from place
        osmn_graph_obj = None
        try:
            osmn_graph_obj = ox.graph_from_place(
                query=self.query, simplify=True, network_type="drive"
            )
            if persist:
                self.persist_ograph(osmn_graph_obj)

        except Exception as e:
            raise LookupError(
                f"No graph found for query {self.query}."
                f" Please try a geocode-able place from OpenStreetMaps."
            ) from e
        return osmn_graph_obj

    def project_axis(self):
        """
        projects graph geometry and plots figure, retrieving an axis
        :return: self.fig, self.axis, ax, graph
        """
        # project and plot
        graph = ox.project_graph(self.init_graph)
        fig, ax = ox.plot_graph(
            graph,
            node_size=0,
            edge_linewidth=0.5,
            show=bool(self.preview),
            bgcolor="#FFFFFF",
        )
        # set the axis title and grab the dimensions of the figure
        self.fig = fig
        ax.set_title(self.query)
        self.axis = ax.axis()
        return ax, graph

    def persist_ograph(self, osmn_graph_obj):
        ox.save_graphml(osmn_graph_obj, filepath=self.path_to_saved_osmnx_graph_obj)

    @property
    def path_to_saved_osmnx_graph_obj(self) -> pathlib.Path:
        return pathlib.Path(f"{self.path_to_graphml_files}/{self.graph_name}")
