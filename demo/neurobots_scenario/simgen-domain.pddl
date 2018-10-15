(define (domain neurobots-simgen)
  (:requirements :adl :object-fluents)
  (:types
   vase furniture robot human - base
   shelf table flowerbed - furniture
   vessel flower - transportable
   vase drinkingvessel - vessel
   bottle odv - drinkingvessel
   glass cup - odv
   rose tulip sunflower - flower
   flower - content
   shape
   room   
   color
   alignment)

  (:constants
   empty - content
   nowhere - base
   uncolored - color)

  (:predicates
   (connected ?l1 ?l2 - room)
   (is-open ?v - vessel)
   (arm-empty ?r - robot)
   (mobile ?r - robot) ;; only mobile robots can move around
   (has-approached ?r - robot ?newbase - base)
   (has-arranged ?fl - flower ?v - vase)
   (has-drunk ?h - human ?g - drinkingvessel ?c - content)
   (has-dropped ?dropper - robot ?t - transportable)
   (has-given ?t - transportable ?h - human)
   (went-to ?r - robot ?to - room)
   (has-grasped ?rob - robot ?t - transportable)
   (has-opened ?r - robot ?b - bottle)
   (has-picked ?fl - flower ?v - vase)
   (has-poured ?o1 - drinkingvessel ?o2 - drinkingvessel ?c - content))

  (:functions
   (position ?t - transportable) - base
   (at ?r - robot) - base
   (in ?b - base) - room
   (aligned ?f - furniture) - alignment
   (contains ?v - vessel) - content
   (shaped ?g - glass) - shape
   (colored ?o - object) - color)

  (:action approach
 	  :parameters (?r - robot ?oldbase ?newbase - base ?rm - room)
 	  :precondition (and (= (at ?r) ?oldbase)
 	  					 (= (in ?r) ?rm)
 	  					 (= (in ?newbase) ?rm)
 	  					 (not (= ?newbase ?r)))
 	  :effect (and (assign (at ?r) ?newbase)
 	  			   (has-approached ?r ?newbase)))

  (:action arrange
 	  :parameters (?r - robot ?fl - flower ?v - vase ?fu - furniture)
 	  :precondition (and (= (at ?r) ?fu)
                         (= (position ?v) ?fu)
                         (= (position ?fl) ?r)
                         (= (contains ?v) empty))
 	  :effect (and (assign (contains ?v) ?fl)
				   (assign (position ?fl) ?v)
				   (arm-empty ?r)
				   (has-arranged ?fl ?v)))

  (:action drink
 	  :parameters (?drinkassistant - robot ?h - human ?glass - odv ?c - content)
 	  :precondition (and (= (at ?drinkassistant) ?h)
                         (= (position ?glass) ?drinkassistant)
                         (= (contains ?glass) ?c)
                         (not (= ?c empty)))
 	  :effect (and (assign (contains ?glass) empty)
                   (has-drunk ?h ?glass ?c)))

  (:action drop
 	  :parameters (?dropper - robot ?t - transportable ?fu - furniture)
 	  :precondition (and (= (position ?t) ?dropper)
					     (= (at ?dropper) ?fu))
 	  :effect (and (assign (position ?t) ?fu)
	               (arm-empty ?dropper)
	               (has-dropped ?dropper ?t)))

  (:action give
 	  :parameters(?rob - robot ?t - transportable ?h - human)
	  :precondition (and (= (at ?rob) ?h)
	  		     		 (= (position ?t) ?rob))
 	  :effect (and (assign (position ?t) ?h)
	  	           (arm-empty ?rob)
	  	           (has-given ?t ?h)))

  (:action go
 	  :parameters (?r - robot ?from ?to - room ?oldbase - base)
 	  :precondition (and (= (in ?r) ?from)
					     (= (at ?r) ?oldbase)
					     (mobile ?r)
					     (connected ?from ?to))
 	  :effect (and (assign (in ?r) ?to)
           		   (assign (at ?r) nowhere)
           		   (went-to ?r ?to)))

  (:action grasp
 	  :parameters (?rob - robot ?t - transportable ?b - base)
 	  :precondition (and (= (at ?rob) ?b)
                         (= (position ?t) ?b)
                         (arm-empty ?rob))
 	  :effect (and (assign (position ?t) ?rob)
                   (not (arm-empty ?rob))
                   (has-grasped ?rob ?t)))

  (:action open
 	  :parameters (?r - robot ?b - bottle ?f - furniture)
 	  :precondition (and (= (position ?b) ?f)
                         (= (at ?r) ?f)
                         (arm-empty ?r)
                         (not (is-open ?b)))
 	  :effect (and (is-open ?b)
           		   (has-opened ?r ?b)))
                        
  (:action pick
 	  :parameters (?rob - robot ?fl - flower ?v - vase ?fu - furniture)
 	  :precondition (and (= (at ?rob) ?fu)
                         (= (position ?v) ?fu)
                         (= (contains ?v) ?fl)
                         (= (position ?fl) ?v)
                         (arm-empty ?rob))
 	  :effect (and (assign (position ?fl) ?rob)
           		   (assign (contains ?v) empty)
           		   (not (arm-empty ?rob))
           		   (has-picked ?fl ?v)))

  (:action pour
 	  :parameters (?rob - robot ?o1 - drinkingvessel ?o2 - drinkingvessel ?f - furniture ?c - content)
 	  :precondition (and (= (position ?o1) ?rob)
                         (= (position ?o2) ?f)
                         (= (at ?rob) ?f)
                         (is-open ?o1)
                         (is-open ?o2)
                         (= (contains ?o1) ?c)
                         (= (contains ?o2) empty))
 	  :effect (and (assign (contains ?o2) ?c)
                   (assign (contains ?o1) empty)
                   (has-poured ?o1 ?o2 ?c)))
)