# gosearch

## AI search routines

This is a work in progress and more algorithms will be added over time.

For examples on how to use the routines please look at [hduplooy/gosearch-test](https://github.com/hduplooy/gosearch-test).

There are 4 algorithms currently and each are called the same and return the same kind of results:
* [DepthFirstSearch](https://en.wikipedia.org/wiki/Depth-first_search) - For every element each of its descendants are searched first before siblings are done till a goal is found,
* [BreadthFirstSearch](https://en.wikipedia.org/wiki/Breadth-first_search) - For every element its descendants are placed at the back of the queue and the elements siblings are done first until a goal is found,
* [BestCostSearch](https://en.wikipedia.org/wiki/Best-first_search) - For every element its added to the search queue based on the current total cost  to reach this state. The ones with the lowest cost and the estimated distance to the goal is searched first until a goal is found. If the away func returns 0 this reduces to something similar to [Dijkstra's Algorithm](https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm).

The state elements that your implementation uses must adhere to the SearchF interface and implement the following funcs:
* Descendants() - Returns all valid descendants for the current state,
* Done() - Tests if the current state is the goal state,
* Cost() - Returns the total cost to reach this state,
* Away() - Returns the estimated distance from the goal, and
* Key() - Returns a unique string key that identifies the state so that it is not done repeatedly.

Each routine is called with the starting state and a boolean to indicate if the history must be returned (the states from the start to goal), these then return the number of steps used to find the goal, the actual goal state and the history (if specified else nil).


