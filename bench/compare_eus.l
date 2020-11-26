(load "package://fetcheus/fetch-interface.l") 
(fetch)

(setq link (send *fetch* :rarm :end-coords))
(setq move-targets
      (mapcar #'(lambda (m) (send *fetch* m)) 
              `(:shoulder_pan_link_lk 
                 :shoulder_lift_link_lk 
                 :upperarm_roll_link_lk
                 :elbow_flex_link_lk
                 :forearm_roll_link_lk
                 :wrist_flex_link_lk
                 :wrist_roll_link_lk)))

(setq link (cddr (send *fetch* :link-list (send (send *fetch* :rarm :end-coords) :parent))))
(bench (dotimes (i 1000)
         ;; set joint angles:
         (mapcar #'(lambda (j) (send *fetch* j :joint-angle 0))
                 `(
                   :shoulder_pan_joint
                   :shoulder_lift_joint
                   :upperarm_roll_joint
                   :elbow_flex_joint
                   :wrist_flex_joint
                   :wrist_roll_joint))
         ;; compute jacobian for each collision links
         (dolist (mt move-targets)
             (setq jac
                   (send *fetch* :calc-jacobian-from-link-list link
                         :move-target mt
                         :rotation-axis nil
                         :translation-axis t)))
         ))
