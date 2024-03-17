---
layout: post
title:  "Write and render peseudocode in the blogs"
date:   2024-03-16 20:05:00 +0800
tags: 
  - js
categories:
  - site building
---

The way to write and render pseudocode in personal site  will be introduced in this blog.

### I. Using MathJax 3.x to render formula

Include the following in the `<head>` of your page:

```html
<script>
    MathJax = {
        tex: {
            inlineMath: [['$','$'], ['\\(','\\)']],
            displayMath: [['$$','$$'], ['\\[','\\]']],
            processEscapes: true,
            processEnvironments: true,
        }
    }
</script>
<script src="https://cdn.jsdelivr.net/npm/mathjax@3.2.2/es5/tex-chtml-full.js"
        integrity="sha256-kbAFUDxdHwlYv01zraGjvjNZayxKtdoiJ38bDTFJtaQ="
        crossorigin="anonymous">
</script>
```

### II. Grab pseudocode.js

Include the following in the `<head>` of your page:

```html
<link rel="stylesheet" href="https://cdn.jsdelivr.net/npm/pseudocode@2.4.1/build/pseudocode.min.css">
<script src="https://cdn.jsdelivr.net/npm/pseudocode@2.4.1/build/pseudocode.min.js">
</script>
```

### III. Write your pseudocode inside a `<pre>`

We assume the pseudocode to be rendered is in a `<pre>` DOM element. Here is an example that illustrates a quicksort algorithm:

```html
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
```

### IV. Render the element or class using pseudocode.js

Insert the following Javascript snippet at the end of your document:

- Render the element using pseudocode.js
    ```html
    <script>
        pseudocode.renderElement(document.getElementById("quicksort"));
    </script>
    ```
- Render all elements of the class using pseudocode.js
    ```html
    <script>
        pseudocode.renderClass("pseudocode");
    </script>
    ```

### Example
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


### Reference
- [`pseudocode.js` Quick Start](https://github.com/SaswatPadhi/pseudocode.js/blob/master/README.md)
- [Beautiful pseudocode for the Web](https://saswat.padhi.me/pseudocode.js/)
- [如何在博客中插入算法伪代码](https://zjuguoshuai.gitlab.io/2019/04/26/blog-pseudocode.html)
