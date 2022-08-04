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
        id = f"{self.node_to_render.lower()}-{uuid.uuid4()}"
        properties = self.__parse_properties()

        content = self.child_nodes.render(context)

        if 'react-components' not in context.render_context:
            context.render_context['react-components'] = []

        context.render_context['react-components'].append((id, self.node_to_render, properties))

        return f"<div id='{id}'>{str(content)}</div>"


@register.tag(name="react_components_render")
def do_react_components_render(parser, token):
    return ReactComponentsRenderer()


class ReactComponentsRenderer(template.Node):
    def __init__(self):
        pass

    def render(self, context):
        components = ""
        for react_component in context.render_context['react-components']:
            component = f"['{react_component[1]}', " \
                        f"document.getElementById('{react_component[0]}')," \
                        f"{json.dumps(react_component[2])}],"
            components += f"{component}\n"
        return f"""
<script>
    RoboticsExerciseComponents.render([{components}]);
</script>            
"""
