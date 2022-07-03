import uuid

from django import template

register = template.Library()


@register.tag(name="react_component")
def do_react_component(parser, token):
    tag_name, node_name = token.split_contents()
    return ReactComponent(node_name)


class ReactComponent(template.Node):
    def __init__(self, node_to_render):
        self.node_to_render = node_to_render

    def render(self, context):
        id = f"{self.node_to_render.lower()}-{uuid.uuid4()}"

        if 'react-components' not in context:
            context.render_context['react-components'] = []

        context.render_context['react-components'].append((id, self.node_to_render))
        return f"<div id='{id}'></div>"


@register.tag(name="react_components_render")
def do_react_components_render(parser, token):
    return ReactComponentsRenderer()


class ReactComponentsRenderer(template.Node):
    def __init__(self):
        pass

    def render(self, context):
        components = ""
        for react_component in context.render_context['react-components']:
            component = f"RoboticsExerciseComponents.render('{react_component[1]}', " \
                        f"document.getElementById('{react_component[0]}'));"
            components += f"{component}\n"
        return f"""
<script>
{components}
</script>            
"""
