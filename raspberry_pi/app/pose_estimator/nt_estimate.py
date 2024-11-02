"""Read measurements from Network Tables, run the
smoother, and publish the results on Network Tables."""


from app.network.network_protocol import Network


class NTEstimate:
    def __init__(self, net: Network) -> None:
        self.net = net

        
