# Build the thesis with XeLaTeX.
#
# TinyTeX is installed under $HOME and is not on the default PATH, so it is
# added here rather than relying on the caller's shell.
TEXBIN := $(HOME)/.TinyTeX/bin/x86_64-linux
export PATH := $(TEXBIN):$(HOME)/.local/bin:$(PATH)

MAIN := main

.PHONY: all clean distclean todos

all: $(MAIN).pdf

# Explicit four-pass sequence rather than latexmk alone, so the bibtex step is
# visible in the log and a missing citation is obvious.
$(MAIN).pdf: $(MAIN).tex $(wildcard */*.tex) bibs/sample.bib
	xelatex -interaction=nonstopmode -file-line-error $(MAIN).tex
	-bibtex $(MAIN)
	xelatex -interaction=nonstopmode -file-line-error $(MAIN).tex
	xelatex -interaction=nonstopmode -file-line-error $(MAIN).tex
	@echo "--- build finished: $(MAIN).pdf ---"

todos:
	@grep -rno '\\todo{[^}]*}' --include='*.tex' . | sed 's/^\.\///'

clean:
	rm -f *.aux */*.aux *.log *.bbl *.blg *.out *.toc *.fdb_latexmk *.fls *.synctex.gz *.lof *.lot

distclean: clean
	rm -f $(MAIN).pdf
