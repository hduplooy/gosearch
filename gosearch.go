// github.com/hduplooy/gosearch
// Author: Hannes du Plooy
// Revision Date: 3 Sep 2016
// Revision Date: 8 Sep 2016 - Implemented priority queues for BestCostSearch
// AI search routines in go
// This is a work in progress and not exhaustive
package gosearch

import (
	"container/heap"
)

// searchElement is used by the routines to keep a queue of elements that need to be processed
// Next points to the next element in the queue
// Prev points to the previous element
// Parent is the element that produced this elements
// Data is the actual data that indicates the state of this element (it is an interface that
// the user of the library must adhere to)
type searchElement struct {
	Next   *searchElement
	Prev   *searchElement
	Parent *searchElement
	Data   SearchF
}

// SearchF is an interface that the user of the library must adhere to
// Descendants will return valid descendants of the current element
// Done will return true if this is the goal state
// Cost is used by the BestCost and BestCostAway and should return the total cost to move to the current element
// Away is used by the BestCostAway and provide an estimate of how far away we are from the goal
// Key is an unique string key so that we can stop the routines from executing the same element several times
type SearchF interface {
	Descendants() []SearchF
	Done() bool
	Cost() float64
	Away() float64
	Key() string
}

// NewElement just return a new searchElement with the data in it
func NewElement(data SearchF) *searchElement {
	return &searchElement{Data: data}
}

// DepthFirstSearch will start the search and search until the Done() func returns true
// If keephistory is true the elements that led to the goal state is also returned
// the number of steps to the goal, the goal and the history is returned
func DepthFirstSearch(data SearchF, keephistory bool) (int, SearchF, []SearchF) {
	// map to check if elements are already being processed or were processed
	pres := make(map[string]bool)
	// The head of the queue start with the first element provided
	head := NewElement(data)
	cnt := 0
	// while there is still something to do
	for head != nil {
		cnt++
		// Get the first one in the queue
		cur := head
		// Remove from queue
		head = head.Next
		// If it is done then generate the history (if any) and return the steps, goal and history
		if cur.Data.Done() {
			hist := make([]SearchF, 0, 10)
			tmp := cur.Parent
			for i := 0; tmp != nil; i, tmp = i+1, tmp.Parent {
				hist = append(hist, tmp.Data)
			}
			return cnt, cur.Data, hist
		}
		// If not done - get the key for the element and if not present process it
		key := cur.Data.Key()
		if _, ok := pres[key]; !ok {
			// Get all the valid descendants and put in front of queue (this is what makes it depth first search)
			for _, val := range cur.Data.Descendants() {
				tmp := NewElement(val)
				if keephistory {
					tmp.Parent = cur
				}
				tmp.Next = head
				head = tmp
			}
		}
	}
	// No answer
	return cnt, nil, nil
}

// BreadthFirstSearch is similar to to DepthFirstSearch except that everything is placed at the back of the queue instead of in front
// BreadthFirstSearch will start the search and search until the Done() func returns true
// If keephistory is true the elements that led to the goal state is also returned
// the number of steps to the goal, the goal and the history is returned
func BreadthFirstSearch(data SearchF, keephistory bool) (int, SearchF, []SearchF) {
	// map to check if elements are already being processed or were processed
	pres := make(map[string]bool)
	// The head of the queue start with the first element provided
	head := NewElement(data)
	// Because we place stuff at the end of the queue we need to keep track of the tail
	tail := head
	cnt := 0
	// while there is still something to do
	for head != nil {
		cnt++
		// Get the first one in the queue
		cur := head
		// Remove from queue
		head = head.Next
		// If the head is nill it means that the tail must be nil as well
		if head == nil {
			tail = nil
		}
		// If it is done then generate the history (if any) and return the steps, goal and history
		if cur.Data.Done() {
			hist := make([]SearchF, 0, 10)
			tmp := cur.Parent
			for i := 0; tmp != nil; i, tmp = i+1, tmp.Parent {
				hist = append(hist, tmp.Data)
			}
			return cnt, cur.Data, hist
		}
		// If not done - get the key for the element and if not present process it
		key := cur.Data.Key()
		if _, ok := pres[key]; !ok {
			// Get all the valid descendants and put at back of queue (this is what makes it breadth first search)
			for _, val := range cur.Data.Descendants() {
				tmp := NewElement(val)
				if keephistory {
					tmp.Parent = cur
				}
				if tail != nil {
					tail.Next = tmp
				}
				tail = tmp
				if head == nil {
					head = tmp
				}
			}
		}
	}
	return cnt, nil, nil
}

// costElement is used in BestCostSearch as an element in a minimum priority queue
// The standard golang heap is used to implement the priority queue
type costElement struct {
	// The parent if we want to record the history
	Parent *costElement
	// The interface that will give us the cost and away
	Data SearchF
	// Used in the priority queue to fix the position
	Index int
}

// costPriorityQueue is a interface definition to implement the heap.Interface
type costPriorityQueue []*costElement

// Len returns the number of elements in the queue
func (pq costPriorityQueue) Len() int { return len(pq) }

// Less returns if the first element is less than the second
// The cost + the estimated distance is used in the comparison
// If return a 0 with the Away func then it reduces sort of to Dijkstra's Algorithm
func (pq costPriorityQueue) Less(i, j int) bool {
	return pq[i].Data.Cost()+pq[i].Data.Away() < pq[j].Data.Cost()+pq[j].Data.Away()
}

// Swap will swap the entries in the queue
func (pq costPriorityQueue) Swap(i, j int) {
	pq[i], pq[j] = pq[j], pq[i]
	pq[i].Index = i
	pq[j].Index = j
}

// Push pushes a value on the back of the array
// heap.Push calls this and then sorts out where it really should go in the queue
func (pq *costPriorityQueue) Push(x interface{}) {
	n := len(*pq)
	item := x.(*costElement)
	item.Index = n
	*pq = append(*pq, item)
}

// Pop returns the last element in the queue which should be the lowest value
// heap.Pop calls this and then sorts out the queue if it should be
func (pq *costPriorityQueue) Pop() interface{} {
	old := *pq
	item := old[len(old)-1]
	item.Index = -1
	*pq = old[:len(old)-1]

	return item
}

// BestCostSearch is similar to to DepthFirstSearch except that a priority queue is kept in place with the lowest cost+away elements earlier in the queue
// BestCostSearch will start the search and search until the Done() func returns true
// If keephistory is true the elements that led to the goal state is also returned
// the number of steps to the goal, the goal and the history is returned
func BestCostSearch(data SearchF, keephistory bool) (int, SearchF, []SearchF) {
	// map to check if elements are already being processed or were processed
	pres := make(map[string]bool)
	// Initializes heap
	pq := make(costPriorityQueue, 0, 100)
	heap.Init(&pq)
	// Push start state on queue
	heap.Push(&pq, &costElement{Data: data})
	cnt := 0
	// while there is still something to do
	for pq.Len() > 0 {
		cnt++
		// Get the lowest cost+away in the queue and remove
		cur := heap.Pop(&pq).(*costElement)
		// If it is done then generate the history (if any) and return the steps, goal and history
		if cur.Data.Done() {
			hist := make([]SearchF, 0, 10)
			tmp := cur.Parent
			for i := 0; tmp != nil; i, tmp = i+1, tmp.Parent {
				hist = append(hist, tmp.Data)
			}
			return cnt, cur.Data, hist
		}
		// If not done - get the key for the element and if not present process it
		key := cur.Data.Key()
		if _, ok := pres[key]; !ok {
			// Get all the valid descendants and put it in queue based on it's cost
			descendants := cur.Data.Descendants()
			for _, val := range descendants {
				newel := &costElement{Data: val}
				if keephistory {
					newel.Parent = cur
				}
				// Push back on priority queue based on the cost+away
				heap.Push(&pq, newel)
			}
		}
	}
	return cnt, nil, nil
}
