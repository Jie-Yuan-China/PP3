#include "AStar.hpp"
#include <algorithm>

using namespace std::placeholders;

bool AStar::Vec2i::operator ==(const Vec2i& coordinates_) {
	return (x == coordinates_.x && y == coordinates_.y);
}

AStar::Vec2i operator +(const AStar::Vec2i& left_, const AStar::Vec2i& right_) {
	return {left_.x + right_.x, left_.y + right_.y};
}

AStar::Node::Node(Vec2i coordinates_, Node *parent_) {
	parent = parent_;
	coordinates = coordinates_;
	G = H = 0;
}

AStar::uint AStar::Node::getScore() {
	return G + H;
}

AStar::Generator::Generator() {
	setDiagonalMovement(false);
	setHeuristic(&Heuristic::manhattan);
	direction = {
		{	0, 1}, {1, 0}, {0, -1}, {-1, 0},
		{	-1, -1}, {1, 1}, {-1, 1}, {1, -1}
	};
}

void AStar::Generator::setWorldSize(Vec2i worldSize_) {
	worldSize = worldSize_;
}

void AStar::Generator::setDiagonalMovement(bool enable_) {
	directions = (enable_ ? 8 : 4);
}

void AStar::Generator::setHeuristic(HeuristicFunction heuristic_) {
	heuristic = std::bind(heuristic_, _1, _2);
}

void AStar::Generator::addCollision(Vec2i coordinates_) {
	walls.push_back(coordinates_);
	uint costs=34;
	int buffersize=2;
	for(int i=-buffersize;i<=buffersize;i++){
		for(int j=-buffersize;j<=buffersize;j++){
			if (i == 0 && j == 0)
				continue;
			Vec2i v;
			v.x=coordinates_.x+i;
			v.y=coordinates_.y+j;
			v.cost=costs - std::min(abs(i),abs(j))*6;
			if (detectCollision(v)){
				continue;
			}
			buffer.push_back(v);
		}
	}
//    neighbours
//    buffer.push_back()
}

void AStar::Generator::removeCollision(Vec2i coordinates_) {
	auto it = std::find(walls.begin(), walls.end(), coordinates_);
	if (it != walls.end()) {
		walls.erase(it);
	}
	auto i = std::find(buffer.begin(), buffer.end(), coordinates_);
	if (i != buffer.end()) {
		buffer.erase(i);
	}
}

void AStar::Generator::clearCollisions() {
	walls.clear();
	buffer.clear();
}

AStar::CoordinateList AStar::Generator::findPath(Vec2i source_, Vec2i target_) {
	Node *current = nullptr;
	NodeSet openSet, closedSet;
	openSet.insert(new Node(source_));

	while (!openSet.empty()) {
		current = *openSet.begin();
		for (auto node : openSet) {
			if (node->getScore() <= current->getScore()) {
				current = node;
			}
		}

		if (current->coordinates == target_) {
			break;
		}

		closedSet.insert(current);
		openSet.erase(std::find(openSet.begin(), openSet.end(), current));

		for (uint i = 0; i < directions; ++i) {
			Vec2i newCoordinates(current->coordinates + direction[i]);
			if (detectCollision(newCoordinates)
					|| findNodeOnList(closedSet, newCoordinates)) {
				continue;
			}
			uint totalCost = current->G;
			std::vector<Vec2i>::iterator it=find(buffer.begin(),buffer.end(),newCoordinates);

			if (it	!= buffer.end()) {
				totalCost += it->cost; //todo find the cost of buffer
			} else {
				totalCost += ((i < 4) ? 10 : 14);
			}
			Node *successor = findNodeOnList(openSet, newCoordinates);
			if (successor == nullptr) {
				successor = new Node(newCoordinates, current);
				successor->G = totalCost;
				successor->H = heuristic(successor->coordinates, target_);
				openSet.insert(successor);
			} else if (totalCost < successor->G) {
				successor->parent = current;
				successor->G = totalCost;
			}
		}
	}

	CoordinateList path;
	while (current != nullptr) {
		path.push_back(current->coordinates);
		current = current->parent;
	}

	releaseNodes(openSet);
	releaseNodes(closedSet);

	return path;
}

AStar::Node* AStar::Generator::findNodeOnList(NodeSet& nodes_,
		Vec2i coordinates_) {
	for (auto node : nodes_) {
		if (node->coordinates == coordinates_) {
			return node;
		}
	}
	return nullptr;
}

void AStar::Generator::releaseNodes(NodeSet& nodes_) {
	for (auto it = nodes_.begin(); it != nodes_.end();) {
		delete *it;
		it = nodes_.erase(it);
	}
}

bool AStar::Generator::detectCollision(Vec2i coordinates_) {
	if (coordinates_.x < 0 || coordinates_.x >= worldSize.x
			|| coordinates_.y < 0 || coordinates_.y >= worldSize.y
			|| std::find(walls.begin(), walls.end(), coordinates_)
					!= walls.end()) {
		return true;
	}
	return false;
}

AStar::Vec2i AStar::Heuristic::getDelta(Vec2i source_, Vec2i target_) {
	return {abs(source_.x - target_.x), abs(source_.y - target_.y)};
}

AStar::uint AStar::Heuristic::manhattan(Vec2i source_, Vec2i target_) {
	auto delta = std::move(getDelta(source_, target_));
	return static_cast<uint>(10 * (delta.x + delta.y));
}

AStar::uint AStar::Heuristic::euclidean(Vec2i source_, Vec2i target_) {
	auto delta = std::move(getDelta(source_, target_));
	return static_cast<uint>(10 * sqrt(pow(delta.x, 2) + pow(delta.y, 2)));
}

AStar::uint AStar::Heuristic::octagonal(Vec2i source_, Vec2i target_) {
	auto delta = std::move(getDelta(source_, target_));
	return 10 * (delta.x + delta.y) + (-6) * std::min(delta.x, delta.y);
}
