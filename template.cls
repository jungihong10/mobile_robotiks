\ProvidesClass{template} \RequirePackage{ifthen}
\usepackage[utf8]{inputenc}   % für Umlaute
\newboolean{@german} \setboolean{@german}{false}
\DeclareOption{german}{\setboolean{@german}{true}}
\DeclareOption*{\PassOptionsToClass{\CurrentOption}{scrartcl}} \ProcessOptions
\LoadClass[a4paper]{scrartcl} \usepackage{epsfig}

% Einstellungen der Seitenränder
\usepackage[left=2cm,right=2cm,top=3cm,bottom=3cm,includeheadfoot]{geometry}

\usepackage{ifxetex}
\ifxetex
    % Only if compiling with XeLaTex
    \usepackage{polyglossia}
   \setmainlanguage[spelling=new,babelshorthands=true]{german}
    \usepackage{fontspec}
    \usepackage[]{unicode-math}
    \usepackage{amsmath}
\else
    % Default: pdfLaTeX
    % neue deutsche Rechtschreibung
    \usepackage{german}
    \usepackage{ngerman}
    \usepackage[utf8]{inputenc}
    \usepackage{amsmath}
\fi

\usepackage{longtable}
\usepackage{epstopdf}

\usepackage{tikz}
\usepackage{pgfplots}
\usepackage{pgfplotstable}
\pgfplotsset{compat=1.6}
\usetikzlibrary{arrows,automata}

\usepackage{xcolor}
\definecolor{dkgreen}{rgb}{0,0.6,0}
\definecolor{dred}{rgb}{0.545,0,0}
\definecolor{dblue}{rgb}{0,0,0.545}
\definecolor{lgrey}{rgb}{0.95,0.95,0.95}
\definecolor{gray}{rgb}{0.4,0.4,0.4}
\definecolor{darkblue}{rgb}{0.0,0.0,0.6}
\usepackage{listings}
\lstset{literate=%
    {Ö}{{\"O}}1
    {Ä}{{\"A}}1
    {Ü}{{\"U}}1
    {ß}{{\ss}}1
    {ü}{{\"u}}1
    {ä}{{\"a}}1
    {ö}{{\"o}}1
    {~}{{\textasciitilde}}1,
    basicstyle=\small\ttfamily,
    keywordstyle=\bfseries,
    language=[LaTeX]{TeX},
    backgroundcolor=\color{lgrey},  
    basicstyle=\footnotesize \ttfamily \color{black} \bfseries,   
    breakatwhitespace=false,       
    breaklines=true,               
    captionpos=b,                   
    commentstyle=\color{dkgreen},   
    deletekeywords={...},          
    escapeinside={\%*}{*)},                  
    frame=single,                  
    language=C++,                
    keywordstyle=\color{purple},  
    morekeywords={BRIEFDescriptorConfig,string,TiXmlNode,DetectorDescriptorConfigContainer,istringstream,
		  cout,endl,cerr,exit,std}, 
    identifierstyle=\color{black},
    stringstyle=\color{blue},      
    numbers=right,                 
    numbersep=6pt,                  
    numberstyle=\tiny\color{black}, 
    rulecolor=\color{black},        
    showspaces=false,               
    showstringspaces=false,        
    showtabs=false,                
    stepnumber=1,                   
    tabsize=3,                     
    title=\lstname,          
}
\usepackage{fixltx2e}
\usepackage{float}
\usepackage{graphicx}

% Kopf- und Fußzeile
\usepackage{fancyhdr}
\pagestyle{fancy}

\usepackage{setspace}
% Kopfzeile links bzw. innen
\fancyhead[L]{
\parbox{0.28\linewidth}{\setstretch{0.3}
\nameA
}\\
\parbox{0.28\linewidth}{\setstretch{0.3}
\nameB
}\\
\parbox{0.28\linewidth}{\setstretch{0.3}
\nameC % Hier die dritte Person hinzufügen
}
}

% Kopfzeile mittig
\fancyhead[C]{
{\bf \large Lab Course Mobile Roboter}\\
{\bf Assignment \uNr, 
\semester}
}
% Kopfzeile rechts bzw. außen
\fancyhead[R]{Group \gNr
\\
\today}
% Linie oben
\renewcommand{\headrulewidth}{0.5pt}
% Linie unten
\renewcommand{\footrulewidth}{0.0pt}
% Fußzeile mittig
\fancyfoot[R]{\thepage}
\fancyfoot[C]{}
\fancyfoot[L]{{Commit: \gitCommit}}
