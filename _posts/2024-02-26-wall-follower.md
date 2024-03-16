---
layout: post
title:  "Wall Follower"
date:   2024-02-26 19:05:00 +0800
tags: 
    - motion planning
    - quadcopter drone
categories:
---

如何让一架无人机跟随墙壁飞行呢？

<pre id="quicksort" class="pseudocode">
    % This quicksort algorithm is extracted from Chapter 7, Introduction to Algorithms (3rd edition)
    \begin{algorithm}
    \caption{Quicksort}
    \begin{algorithmic}
    \PROCEDURE{Quicksort}{$A, p, r$}
        \IF{$p < r$} 
            \STATE $q = $ \CALL{Partition}{$A, p, r$}
            \STATE \CALL{Quicksort}{$A, p, q - 1$}
            \STATE \CALL{Quicksort}{$A, q + 1, r$}
        \ENDIF
    \ENDPROCEDURE
    \PROCEDURE{Partition}{$A, p, r$}
        \STATE $x = A[r]$
        \STATE $i = p - 1$
        \FOR{$j = p$ \TO $r - 1$}
            \IF{$A[j] < x$}
                \STATE $i = i + 1$
                \STATE exchange
                $A[i]$ with $A[j]$
            \ENDIF
            \STATE exchange $A[i]$ with $A[r]$
        \ENDFOR
    \ENDPROCEDURE
    \end{algorithmic}
    \end{algorithm}
</pre>

<pre id="test-basics" class="pseudocode">
\begin{algorithm}
\caption{Test text-style}
\begin{algorithmic}
\REQUIRE some preconditions
\ENSURE some postconditions
\INPUT some inputs
\OUTPUT some outputs
\PROCEDURE{Test-Declarations}{}
    \STATE font families: {\sffamily sffamily, \ttfamily ttfamily, \normalfont normalfont, \rmfamily rmfamily.}
    \STATE font weights: {normal weight, \bfseries bold, \mdseries
    medium, \lfseries lighter. }
    \STATE font shapes: {\itshape itshape \scshape Small-Caps \slshape slshape \upshape upshape.}
    \STATE font sizes: \tiny tiny \scriptsize scriptsize \footnotesize
    footnotesize \small small \normalsize normal \large large \Large Large
    \LARGE LARGE \huge huge \Huge Huge \normalsize
\ENDPROCEDURE
\PROCEDURE{Test-Commands}{}
    \STATE \textnormal{textnormal,} \textrm{textrm,} \textsf{textsf,} \texttt{texttt.}
    \STATE \textbf{textbf,} \textmd{textmd,} \textlf{textlf.}
    \STATE \textup{textup,} \textit{textit,} \textsc{textsc,} \textsl{textsl.}
    \STATE \uppercase{uppercase,} \lowercase{LOWERCASE.}
\ENDPROCEDURE
\PROCEDURE{Test-Colors}{}
% feature not implemented
\ENDPROCEDURE
\end{algorithmic}
\end{algorithm}

\begin{algorithm}
\caption{Test atoms}
\begin{algorithmic}
\STATE \textbf{Specials:} \{ \} \$ \& \# \% \_
\STATE \textbf{Bools:} \AND \OR \NOT \TRUE \FALSE
\STATE \textbf{Carriage return:} first line \\ second line
\STATE \textbf{Text-symbols:} \textbackslash
\STATE \textbf{Quote-symbols:} `single quotes', ``double quotes''
\STATE \textbf{Math:} $(\mathcal{C}_m)$, $i \gets i + 1$, $E=mc^2$, \( x^n + y^n = z^n \), $\$$, \(\$\)
\END{ALGORITHMIC}
\END{ALGORITHM}
</pre>

- [Project 2: Robot Wall Following by Reinforcement Learning](https://hcr.cs.umass.edu/courses/compsci603/projects/Compsci_603_Project2_WF.pdf)
- [Chapter 9. Motion Planning in Simple Geometric Spaces](http://motion.cs.illinois.edu/RoboticSystems/GeometricMotionPlanning.html)

