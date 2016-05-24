
#pragma once

class ORBSet
{
public:
	struct Node {
		struct Node *next;
		const char *node_name;
	};

	ORBSet() :
		_top(nullptr),
		_end(nullptr)
	{ }
	~ORBSet()
	{
		while (_top != nullptr) {
			unlinkNext(_top);

			if (_top->next == nullptr) {
				free((void *)_top->node_name);
				free(_top);
				_top = nullptr;
			}
		}
	}
	void insert(const char *node_name)
	{
		Node **p;

		if (_top == nullptr) {
			p = &_top;

		} else {
			p = &_end->next;
		}

		*p = (Node *)malloc(sizeof(Node));

		if (_end) {
			_end = _end->next;

		} else {
			_end = _top;
		}

		_end->next = nullptr;
		_end->node_name = strdup(node_name);
	}

	bool find(const char *node_name)
	{
		Node *p = _top;

		while (p) {
			if (strcmp(p->node_name, node_name) == 0) {
				return true;
			}

			p = p->next;
		}

		return false;
	}

	bool erase(const char *node_name)
	{
		Node *p = _top;

		if (_top && (strcmp(_top->node_name, node_name) == 0)) {
			p = _top->next;
			free((void *)_top->node_name);
			free(_top);
			_top = p;

			if (_top == nullptr) {
				_end = nullptr;
			}

			return true;
		}

		while (p->next) {
			if (strcmp(p->next->node_name, node_name) == 0) {
				unlinkNext(p);
				return true;
			}
		}

		return false;
	}

private:

	void unlinkNext(Node *a)
	{
		Node *b = a->next;

		if (b != nullptr) {
			if (_end == b) {
				_end = a;
			}

			a->next = b->next;
			free((void *)b->node_name);
			free(b);
		}
	}

	Node *_top;
	Node *_end;
};

