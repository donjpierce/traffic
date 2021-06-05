import osmnx as ox
import os


class OGraph:
    def __init__(self, query, save=False):
        """
        Silently sends a geo-codable query to OSMNX for the graph of roads, saves result,
        and loads .graphml files locally if they exist in the graph_files/ folder

        :param query: str: geo-codable query such as "Harlem, NY" or "Kigali, Rwanda"
        :param save: bool:
        """
        self.query = query
        self.save = save
        self.store = 'graphml_files'
        self.graph_name = self.query.lower().replace(',', '').replace(' ', '_') + '.graphml'
        self.local_files = os.listdir('.')
        self.init_graph = self.request() if self.graph_name in self.local_files else ox.load_graphml(self.graph_name)
        self.fig, self.axis = None, None
        self.ax, self.G = self.project_axis()

    def request(self):
        """
        Request a graph from OSMNX

        :return:
        """
        # query graph from place
        G = ox.graph_from_place(query=self.query, simplify=True, network_type='drive')
        if self.save:
            ox.save_graphml(G, filepath=self.graph_name)
        return G

    def project_axis(self):
        """
        projects graph geometry and plots figure, retrieving an axis
        :return: self.fig, self.axis, ax, graph
        """
        # project and plot
        graph = ox.project_graph(self.init_graph)
        fig, ax = ox.plot_graph(graph, node_size=0, edge_linewidth=0.5, show=False)
        # set the axis title and grab the dimensions of the figure
        self.fig = fig
        ax.set_title(self.query)
        self.axis = ax.axis()
        return ax, graph

    def save_graph(self):
        """
        Saves graph to local store
        :return:
        """
        ox.save_graphml(self.G)
        return

    def dir_check(self):
        """
        Create store if does not exist
        :return:
        """
        if not os.path.exists(self.store):
            os.mkdir(self.store)
        return
