import osmnx as ox
import os


class OGraph:
    def __init__(self, query, save=False, preview=False):
        """
        Silently sends a geo-codable query to OSMNX for the graph of roads, saves result,
        and loads .graphml files locally if they exist in the graph_files/ folder

        :param query: str: geo-codable query such as "Harlem, NY" or "Kigali, Rwanda"
        :param preview: bool: will show graph at run-time if True else False
        :param save: bool:
        """
        self.query = query
        self.save = save
        self.preview = preview
        self.store = 'graphml_files'
        self.graph_name = self.query.lower().replace(',', '').replace(' ', '_') + '.graphml'
        self.local_files = os.listdir(self.store)
        self.init_graph = self.request()
        self.fig, self.axis = None, None
        self.ax, self.G = self.project_axis()

    def request(self):
        """
        Negotiates a query to OSMNX if no local stored file else loads local file

        :return: result:
        """
        result = ox.load_graphml(
            self.store + '/' + self.graph_name
        ) if (self.graph_name in self.local_files) else self.send_query()
        return result

    def send_query(self):
        """
        Request a graph from OSMNX

        :return: G: OSMN Graph object
        """
        # query graph from place
        G = None
        try:
            G = ox.graph_from_place(query=self.query, simplify=True, network_type='drive')
            if self.save:
                ox.save_graphml(G, filepath=self.store + '/' + self.graph_name)
        except Exception:
            raise LookupError(f'No graph found for query {self.query}.'
                              f' Please try a geocode-able place from OpenStreetMaps.')
        return G

    def project_axis(self):
        """
        projects graph geometry and plots figure, retrieving an axis
        :return: self.fig, self.axis, ax, graph
        """
        # project and plot
        graph = ox.project_graph(self.init_graph)
        fig, ax = ox.plot_graph(graph, node_size=0, edge_linewidth=0.5,
                                show=True if self.preview else False,
                                bgcolor='#FFFFFF')
        # set the axis title and grab the dimensions of the figure
        self.fig = fig
        ax.set_title(self.query)
        self.axis = ax.axis()
        return ax, graph

    def dir_check(self):
        """
        Create store if does not exist
        :return:
        """
        if not os.path.exists(self.store):
            os.mkdir(self.store)
        return
