package application.ai;

import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Set;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedList;
import application.view.game.HexGrid;
import application.view.game.HexGrid.Tile;

public class Ai {
	// Couleur attribuée à l'IA et au joueur
	private int aiColor = 2;
	private int playerColor = 1;
	// Référence à la grille hexagonale du jeu
	private HexGrid hexGrid;
	// Meilleur coup actuel
	public int[] currentBestMove;
	// Nœud représentant l'absence de chemin disponible
	Node noPathAvailableNode;

	// Compteurs de coupes alpha et bêta
	private int alphaCutoffCounter;
	private int betaCutoffCounter;

	// Constructeur
	public Ai(HexGrid hexGrid) {
		this.hexGrid = hexGrid;
		currentBestMove = new int[2];
		currentBestMove[0] = -2;
		currentBestMove[1] = -2;
		noPathAvailableNode = new Node(Integer.MIN_VALUE, Integer.MIN_VALUE, Integer.MIN_VALUE);

		alphaCutoffCounter = 0;
		betaCutoffCounter = 0;
	}

	// Méthode minimax avec élagage alpha-bêta
	public double minimaxAb(int[][] boardPosition, int depth, double alpha, double beta, boolean isMaximizing) {

		// Récupération des chemins les plus courts pour l'IA et le joueur
		List<Node> computerPath = getComputerShortestPath(boardPosition);
		List<Node> playerPath = getPlayerShortestPath(boardPosition);

		// Si la profondeur est nulle ou qu'aucun chemin n'est disponible pour l'IA ou le joueur, retourner le score heuristique
		if (depth == 0 || computerPath.size() == 0
				|| playerPath.size() == 0)
			return getHeuristicScore(computerPath, playerPath);

		if (isMaximizing) {
			double value = Double.NEGATIVE_INFINITY;

			// Récupération des mouvements possibles pour l'IA
			List<int[]> possibleMoves = hexGrid.getPossibleMoves(boardPosition, aiColor);

			for (int[] move : possibleMoves) {
				int[][] updatedBoard;
				updatedBoard = updateBoard(boardPosition, move, aiColor);

				// Appel récursif à minimax pour évaluer le mouvement
				value = Math.max(value, minimaxAb(updatedBoard, depth - 1, alpha, beta, false));
				if (value > alpha) {
					currentBestMove = move;
				}

				alpha = Math.max(alpha, value);
				if (alpha >= beta) {
					alphaCutoffCounter++;
					break;
				}
			}
			return value;
		} else {
			double value = Double.POSITIVE_INFINITY;

			// Récupération des mouvements possibles pour le joueur
			List<int[]> possibleMoves = hexGrid.getPossibleMoves(boardPosition, playerColor);

			for (int[] move : possibleMoves) {
				int[][] updatedBoard;
				updatedBoard = updateBoard(boardPosition, move, playerColor);

				// Appel récursif à minimax pour évaluer le mouvement
				value = Math.min(value, minimaxAb(updatedBoard, depth - 1, alpha, beta, true));
				beta = Math.min(beta, value);
				if (alpha >= beta) {
					betaCutoffCounter++;
					break;
				}
			}
			return value;
		}
	}

	// Méthode de mise à jour de la position de la grille
	public int[][] updateBoard(int[][] boardPosition, int[] move, int color) {

		int[][] newPosition = new int[hexGrid.getTilesPerRow()][hexGrid.getRowCount()];

		for (int i = 0; i < hexGrid.getRowCount(); i++) {
			for (int j = 0; j < hexGrid.getTilesPerRow(); j++) {
				newPosition[j][i] = boardPosition[j][i];
			}
		}

		newPosition[move[0]][move[1]] = color;
		return newPosition;
	}

	// Méthode de conversion de la grille de tuiles en tableau d'entiers
	public int[][] intFromTile(Tile[][] tileTileCoordArray) {
		int[][] tileCoordArray = new int[hexGrid.getTilesPerRow()][hexGrid.getRowCount()];
		for (int i = 0; i < hexGrid.getRowCount(); i++) {
			for (int j = 0; j < hexGrid.getTilesPerRow(); j++) {
				tileCoordArray[j][i] = tileTileCoordArray[j][i].getStonePlaced();
			}
		}

		return tileCoordArray;
	}

	// Méthode de récupération du score heuristique
	public double getHeuristicScore(List<Node> computerPath, List<Node> playerPath) {

		double computerScore = getScoreForPathComputer(computerPath, playerPath);
		double playerScore = getScoreForPathPlayer(playerPath);

		return playerScore - computerScore;
	}

	// Méthode de récupération du score pour le chemin de l'IA
	private double getScoreForPathComputer(List<Node> computerPath, List<Node> playerPath) {

		if (computerPath.size() == 0.0) {
			return Double.NEGATIVE_INFINITY;
		} else if (computerPath.contains(noPathAvailableNode)) {
			return Double.POSITIVE_INFINITY;
		} else if (playerPath.size() == 0) {
			return Double.POSITIVE_INFINITY;
		} else {
			return computerPath.size();
		}
	}

	// Méthode de récupération du score pour le chemin du joueur
	private double getScoreForPathPlayer(List<Node> path) {

		return path.size();
	}

	// Méthode de récupération du chemin le plus court pour l'IA
	public List<Node> getComputerShortestPath(int[][] tileCoordArray) {

		Graph graph = new Graph(hexGrid, tileCoordArray);
		graph.createGraph(aiColor);

		switch (aiColor) {
		case 1:

			graph = graph.calculateShortestPathFromSource(graph, graph.sourceTop);

			return graph.getShortestPathFilteredList(graph.targetBottom.getShortestPath());

		case 2:

			graph = graph.calculateShortestPathFromSource(graph, graph.sourceLeft);

			return graph.getShortestPathFilteredList(graph.targetRight.getShortestPath());

		}

		return null;
	}

	// Méthode de récupération du chemin le plus court pour le joueur
	public List<Node> getPlayerShortestPath(int[][] tileCoordArray) {
		Graph graph = new Graph(hexGrid, tileCoordArray);
		graph.createGraph(playerColor);

		switch (playerColor) {
		case 1:

			graph = graph.calculateShortestPathFromSource(graph, graph.sourceTop);

			return graph.getShortestPathFilteredList(graph.targetBottom.getShortestPath());

		case 2:

			graph = graph.calculateShortestPathFromSource(graph, graph.sourceLeft);

			return graph.getShortestPathFilteredList(graph.targetRight.getShortestPath());

		}

		return null;
	}

	// Méthode de récupération du compteur de coupes alpha
	public int getAlphaCutoffCounter() {
		return alphaCutoffCounter;
	}

	// Méthode de récupération du compteur de coupes bêta
	public int getBetaCutoffCounter() {
		return betaCutoffCounter;
	}

	// Classe représentant le graphe
	public class Graph {
		private HexGrid hexGrid;
		private int[][] tileCoordArray;
		public Node sourceLeft;
		public Node targetRight;
		public Node sourceTop;
		public Node targetBottom;

		private Set<Node> nodes = new HashSet<>();

		public Graph(HexGrid hexGrid, int[][] tileCoordArray) {
			this.hexGrid = hexGrid;
			this.tileCoordArray = tileCoordArray;
		}

		// Filtre la liste des chemins les plus courts
		public List<Node> getShortestPathFilteredList(List<Node> shortestPath) {
			List<Node> shortestPathFilteredList = new LinkedList<>();
			if (shortestPath.size() != 0) {
				shortestPathFilteredList = shortestPath;

				for (Iterator<Node> i = shortestPathFilteredList.iterator(); i.hasNext();) {
					Node node = i.next();

					if (node.getWeight() == 0)
						i.remove();
				}

				return shortestPathFilteredList;
			} else {
				shortestPathFilteredList.add(noPathAvailableNode);
				return shortestPathFilteredList;
			}
		}

		public int getDistance(int x) {
			if (x == 0)
				return 1;
			else
				return 0;
		}

		public void createGraph(int currentColorPerspective) {
			sourceLeft = new Node(-1, -1, 0);
			targetRight = new Node(-2, -2, 0);
			sourceTop = new Node(-3, -3, 0);
			targetBottom = new Node(-4, -4, 0);

			switch (currentColorPerspective) {
			case 2:
				for (int i = 0; i < hexGrid.getRowCount(); i++) {
					for (int j = 0; j < hexGrid.getTilesPerRow(); j++) {
						if (tileCoordArray[j][i] == currentColorPerspective || tileCoordArray[j][i] == 0) {

							Node node = new Node(j, i, getDistance(tileCoordArray[j][i]));

							if (node.getXHexCoord() == 0)
								sourceLeft.addDestination(node, getDistance(tileCoordArray[j][i]));
							addNode(node);
						}
					}
				}

				for (Node node : nodes) {

					if (node.getXHexCoord() == hexGrid.getTilesPerRow() - 1) {
						node.addDestination(targetRight, targetRight.getWeight());
					}

					for (Node adjNode : nodes) {
						if (adjNode.getXHexCoord() == node.getXHexCoord() + 1
								&& adjNode.getYHexCoord() == node.getYHexCoord() - 1)
							node.addDestination(adjNode, adjNode.getWeight());
					}
					for (Node adjNode : nodes) {
						if (adjNode.getXHexCoord() == node.getXHexCoord() + 1
								&& adjNode.getYHexCoord() == node.getYHexCoord())
							node.addDestination(adjNode, adjNode.getWeight());
					}
					for (Node adjNode : nodes) {
						if (adjNode.getXHexCoord() == node.getXHexCoord()
								&& adjNode.getYHexCoord() == node.getYHexCoord() + 1)
							node.addDestination(adjNode, adjNode.getWeight());
					}

					for (Node adjNode : nodes) {
						if (adjNode.getXHexCoord() == node.getXHexCoord() - 1
								&& adjNode.getYHexCoord() == node.getYHexCoord() + 1)
							node.addDestination(adjNode, adjNode.getWeight());
					}
					for (Node adjNode : nodes) {
						if (adjNode.getXHexCoord() == node.getXHexCoord() - 1
								&& adjNode.getYHexCoord() == node.getYHexCoord())
							node.addDestination(adjNode, adjNode.getWeight());
					}
					for (Node adjNode : nodes) {
						if (adjNode.getXHexCoord() == node.getXHexCoord()
								&& adjNode.getYHexCoord() == node.getYHexCoord() - 1)
							node.addDestination(adjNode, adjNode.getWeight());
					}
				}

				addNode(sourceLeft);
				addNode(targetRight);
				break;
			case 1:
				for (int i = 0; i < hexGrid.getRowCount(); i++) {
					for (int j = 0; j < hexGrid.getTilesPerRow(); j++) {
						if (tileCoordArray[j][i] == currentColorPerspective || tileCoordArray[j][i] == 0) {

							Node node = new Node(j, i, getDistance(tileCoordArray[j][i]));

							if (node.getYHexCoord() == 0)
								sourceTop.addDestination(node, getDistance(tileCoordArray[j][i]));

							addNode(node);
						}
					}
				}

				for (Node node : nodes) {

					if (node.getYHexCoord() == hexGrid.getRowCount() - 1) {
						node.addDestination(targetBottom, targetBottom.getWeight());
					}

					for (Node adjNode : nodes) {
						if (adjNode.getXHexCoord() == node.getXHexCoord() + 1
								&& adjNode.getYHexCoord() == node.getYHexCoord() - 1)
							node.addDestination(adjNode, adjNode.getWeight());
					}
					for (Node adjNode : nodes) {
						if (adjNode.getXHexCoord() == node.getXHexCoord() + 1
								&& adjNode.getYHexCoord() == node.getYHexCoord())
							node.addDestination(adjNode, adjNode.getWeight());
					}
					for (Node adjNode : nodes) {
						if (adjNode.getXHexCoord() == node.getXHexCoord()
								&& adjNode.getYHexCoord() == node.getYHexCoord() + 1)
							node.addDestination(adjNode, adjNode.getWeight());
					}

					for (Node adjNode : nodes) {
						if (adjNode.getXHexCoord() == node.getXHexCoord() - 1
								&& adjNode.getYHexCoord() == node.getYHexCoord() + 1)
							node.addDestination(adjNode, adjNode.getWeight());
					}
					for (Node adjNode : nodes) {
						if (adjNode.getXHexCoord() == node.getXHexCoord() - 1
								&& adjNode.getYHexCoord() == node.getYHexCoord())
							node.addDestination(adjNode, adjNode.getWeight());
					}
					for (Node adjNode : nodes) {
						if (adjNode.getXHexCoord() == node.getXHexCoord()
								&& adjNode.getYHexCoord() == node.getYHexCoord() - 1)
							node.addDestination(adjNode, adjNode.getWeight());
					}
				}

				addNode(sourceTop);
				addNode(targetBottom);
				break;
			}
		}

		public void addNode(Node nodeA) {
			nodes.add(nodeA);
		}

		public Set<Node> getNodes() {
			return nodes;
		}

		// Calcule le chemin le plus court à partir d'un nœud source
		public Graph calculateShortestPathFromSource(Graph graph, Node source) {
			source.setDistance(0);

			Set<Node> settledNodes = new HashSet<>();
			Set<Node> unsettledNodes = new HashSet<>();

			unsettledNodes.add(source);

			while (unsettledNodes.size() != 0) {
				Node currentNode = getLowestDistanceNode(unsettledNodes);
				unsettledNodes.remove(currentNode);
				for (Entry<Node, Integer> adjacencyPair : currentNode.getAdjacentNodes().entrySet()) {
					Node adjacentNode = adjacencyPair.getKey();
					Integer edgeWeight = adjacencyPair.getValue();
					if (!settledNodes.contains(adjacentNode)) {
						calculateMinimumDistance(adjacentNode, edgeWeight, currentNode);
						unsettledNodes.add(adjacentNode);

					}
				}
				settledNodes.add(currentNode);

			}
			return graph;
		}
		// Récupère le nœud avec la plus petite distance parmi un ensemble de nœuds
		private Node getLowestDistanceNode(Set<Node> unsettledNodes) {
			Node lowestDistanceNode = null;
			int lowestDistance = Integer.MAX_VALUE;
			for (Node node : unsettledNodes) {
				int nodeDistance = node.getDistance();
				if (nodeDistance < lowestDistance) {
					lowestDistance = nodeDistance;
					lowestDistanceNode = node;
				}
			}

			return lowestDistanceNode;
		}

		// Calcule la distance minimale pour un nœud évalué
		private void calculateMinimumDistance(Node evaluationNode, Integer edgeWeigh, Node sourceNode) {
			Integer sourceDistance = sourceNode.getDistance();
			if (sourceDistance + edgeWeigh < evaluationNode.getDistance()) {
				evaluationNode.setDistance(sourceDistance + edgeWeigh);
				LinkedList<Node> shortestPath = new LinkedList<>(sourceNode.getShortestPath());
				shortestPath.add(sourceNode);
				evaluationNode.setShortestPath(shortestPath);
			}
		}
	}

	// Classe interne représentant un nœud dans le graphe
	public class Node {
		private int coords;
		private int xHexCoord;
		private int yHexCoord;
		private List<Node> shortestPath = new LinkedList<>();
		private int weight;
		private Integer distance = Integer.MAX_VALUE;

		Map<Node, Integer> adjacentNodes = new HashMap<>();

		public void addDestination(Node destination, int distance) {
			adjacentNodes.put(destination, distance);
		}

		public Node(int xHexCoord, int yHexCoord, int weight) {
			this.xHexCoord = xHexCoord;
			this.yHexCoord = yHexCoord;
			this.coords = xHexCoord * 10 + yHexCoord;
			this.weight = weight;
		}

		public int getWeight() {
			return weight;
		}

		public int getXHexCoord() {
			return xHexCoord;
		}

		public int getYHexCoord() {
			return yHexCoord;
		}

		public int getCoords() {
			return coords;
		}

		public Map<Node, Integer> getAdjacentNodes() {
			return adjacentNodes;
		}

		public int getDistance() {
			return distance;

		}

		public List<Node> getShortestPath() {
			return shortestPath;
		}

		public void setDistance(int distance) {
			this.distance = distance;
		}

		public void setShortestPath(List<Node> shortestPath) {
			this.shortestPath = shortestPath;
		}
	}
}
