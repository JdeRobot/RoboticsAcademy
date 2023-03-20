import json
import uuid

import django.template.base
from django import template

register = template.Library()


@register.tag(name="react_component")
def do_react_component(parser: django.template.base.Parser, token):
    child_nodes = parser.parse(('end_react_component',))
    [tag_name, node_name, *args] = token.split_contents()
    token = parser.next_token()
    return ReactComponent(node_name, child_nodes, args)


class ReactComponent(template.Node):
    def __init__(self, node_to_render, child_nodes, properties):
        self.node_to_render = node_to_render
        self.child_nodes = child_nodes
        self.properties = properties

    def __parse_properties(self):
        properties = {}

        for property in self.properties:
            property = property.split('=')

            if len(property) == 1:
                properties[property[0].strip()] = True
            elif len(property) == 2:
                properties[property[0]] = eval(property[1])
            else:
                raise ValueError("Error in properties")

        return properties

    def render(self, context):
        react_component = {
            "component": self.node_to_render,
            "dom_id": f"{self.node_to_render.lower()}-{uuid.uuid4()}",
            "properties": self.__parse_properties(),
            "children": []
        }

        if 'react-components' not in context.render_context:
            context.render_context['react-components'] = []
            context.render_context['parent-react-component'] = None

        if context.render_context['parent-react-component'] is None:
            context.render_context['react-components'].append(react_component)
            parent_component = None
        else:
            context.render_context['parent-react-component']["children"].append(react_component)
            parent_component = context.render_context['parent-react-component']

        if self.child_nodes:
            context.render_context['parent-react-component'] = react_component
            content = self.child_nodes.render(context)
            context.render_context['parent-react-component'] = parent_component
        else:
            content = ""

        return f"<div id='{id}'>{str(content)}</div>"


@register.tag(name="react_components_render")
def do_react_components_render(parser, token):
    return ReactComponentsRenderer()


class ReactComponentsRenderer(template.Node):
    def __init__(self):
        pass

    # TODO: This function now returns array of components where each one is a javascript prototype, maybe it could
    #  return a root prototype with the list inside it's children array
    def render(self, context):
        components = json.dumps(context.render_context['react-components'])
        return f"""
<script>
    RoboticsExerciseComponents.render([{components}]);
</script>            
"""
