(define (domain switch)
  (:predicates (switch_is_off)
               (switch_is_on))
  
  (:action switch_on
      :parameters ()
      :precondition (switch_is_off)
      :effect (and (switch_is_on)
                   (not (switch_is_off))))
  (:action switch_off
      :parameters ()
      :precondition (switch_is_on)
      :effect (and (switch_is_off)
                   (not (switch_is_on)))))
